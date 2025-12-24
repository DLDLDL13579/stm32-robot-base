#include "AllHeader.h"
#include "mpu6050.h"

// 添加 IMU 数据结构体定义
typedef struct {
    short accel_x;
    short accel_y;
    short accel_z;
    short gyro_x;
    short gyro_y;
    short gyro_z;
    short temp;
} IMUData;

// 全局变量
IMUData imu_data = {0};
// 函数声明
uint32_t GetSystemTime(void);
void ExecuteCommand(char cmd);
void CheckCommandTimeout(void);
void SendEncoderData(void);
void SendIMUData(void);
void CalculateOdometry(void);
void USART1_Init(void);
void SyncSystemTime(uint32_t ros_time_ms);
void ReadEncoderData(int* encoder_data);
#define TX_BUFFER_SIZE 256
char tx_buffer[TX_BUFFER_SIZE];
uint16_t tx_buffer_head = 0;
uint16_t tx_buffer_tail = 0;
uint8_t is_sending = 0; // 标志位，指示DMA是否正在发送
// 配置参数
#define UPLOAD_DATA 1
#define MOTOR_TYPE 1
uint32_t system_time_offset = 0;
uint8_t time_sync_enabled = 0;

// 电机参数
#define PULSE_PER_REVOLUTION (11.0f * 30.0f)
#define WHEEL_DIAMETER 0.067f
#define WHEEL_CIRCUMFERENCE (3.14159f * WHEEL_DIAMETER)
#define DISTANCE_PER_PULSE (WHEEL_CIRCUMFERENCE / PULSE_PER_REVOLUTION)
#define WHEEL_BASE 0.16f
static uint32_t monotonic_time_ms = 0;
static uint32_t last_timestamp = 0;
uint8_t times = 0;
uint8_t upload_counter = 0;
volatile char current_cmd = 0; // 当前正在执行的命令
uint32_t last_cmd_time = 0;
uint32_t last_send_time = 0;
uint32_t last_imu_time = 0;
uint32_t last_calc_time = 0;
#define CMD_TIMEOUT 500
#define SEND_INTERVAL 20    // 编码器数据发送间隔(ms)
#define IMU_INTERVAL 10    // IMU数据发送间隔(ms)

// 在全局变量部分添加以下内容
static int last_valid_encoder_data[4] = {0};
static int16_t last_motor_speeds[4] = {0};
extern int Encoder_Offset[4];
extern int Encoder_Now[4];
static int32_t Encoder_Prev[4] = {0};
extern void control_pwm(int16_t m1,int16_t m2 ,int16_t m3,int16_t m4);

// 机器人状态结构体（简化）
typedef struct {
    float linear_x;
    float linear_y;
    float angular_z;
    float position_x;
    float position_y;
    float orientation;
    int enc1, enc2, enc3, enc4;  // 四个轮子的独立脉冲数
} RobotState;

RobotState robot_state = {0};

// 编码器数据读取函数：读取最新的 32 位绝对脉冲数
void ReadEncoderData(int* encoder_data)
{
    static uint8_t error_count = 0;
    uint8_t retry_count = 0;
    bool data_valid = false;
    
    // 重试机制
    for(retry_count = 0; retry_count < 3; retry_count++) {
        #if UPLOAD_DATA == 1
            // 调用 Read_ALL_Enconder (假设 Read_ALL_Enconder() 返回 32 位绝对脉冲数)
            Read_ALL_Enconder(); 
            for(int i = 0; i < 4; i++) {
                // 读取 32 位绝对脉冲数
                encoder_data[i] = Encoder_Now[i]; 
            }
        #elif UPLOAD_DATA == 2
            // 兼容旧的 16 位偏移量模式
            Read_10_Enconder();
            for(int i = 0; i < 4; i++) {
                encoder_data[i] = Encoder_Offset[i];
            }
        #endif
        
        // 数据有效性检查
        data_valid = true;
        for(int i = 0; i < 4; i++) {
            // 检查异常值：仅检查错误标志 -1
            if(encoder_data[i] == -1) { 
                data_valid = false;
                error_count++;
                break;
            }
        }
        
        if(data_valid) {
            error_count = 0;
            // 更新有效数据
            for(int i = 0; i < 4; i++) {
                last_valid_encoder_data[i] = encoder_data[i];
            }
            break;
        } else {
            // 数据无效，重试前短暂延迟
            delay_ms(1);
        }
    }
    
    // 如果重试后仍然无效，使用上次有效值
    if(!data_valid) {
        for(int i = 0; i < 4; i++) {
            encoder_data[i] = last_valid_encoder_data[i];
        }
        
        static uint32_t last_error_report = 0;
        uint32_t current_time = GetSystemTime();
        if(current_time - last_error_report > 5000) {
            printf("编码器数据异常，使用上次有效值。错误计数: %d\r\n", error_count);
            last_error_report = current_time;
        }
    }
}
void CalculateOdometry(void)
{
    uint32_t current_time = GetSystemTime();
    
    // 1. 读取最新的 32 位绝对脉冲数
    int encoder_now_data[4] = {0}; 
    ReadEncoderData(encoder_now_data); 
    
    // 2. 计算脉冲差值
    int32_t delta_pulse[4]; 

    // 判断是否有运动指令
    bool is_moving_cmd = (current_cmd == 'W' || current_cmd == 'A' || 
                          current_cmd == 'S' || current_cmd == 'D');

    for(int i = 0; i < 4; i++) {
        delta_pulse[i] = encoder_now_data[i] - Encoder_Prev[i]; 

        // =========== 修改开始 ===========
        // 3. 智能死区过滤
        if (!is_moving_cmd) {
            // 如果没有接收到运动指令（静止状态）：
            // 扩大过滤范围到 +/- 5 (甚至可以尝试 10)。
            // 任何小于 5 个脉冲的抖动都会被强制归零，防止 SLAM 地图漂移。
            if(delta_pulse[i] >= -3 && delta_pulse[i] <= 3) {
                delta_pulse[i] = 0;
            }
        }
        // 注意：如果有运动指令 (is_moving_cmd == true)，则不过滤，
        // 保证机器人启动时的微小位移能被记录。
        // =========== 修改结束 ===========

        // 4. 更新上次脉冲数
        Encoder_Prev[i] = encoder_now_data[i]; 
    }
    
    // 5. 存入 robot_state
    robot_state.enc1 = delta_pulse[0];
    robot_state.enc2 = delta_pulse[1];
    robot_state.enc3 = delta_pulse[2];
    robot_state.enc4 = delta_pulse[3];
    
    last_calc_time = current_time;
}
// 发送编码器数据（只发送四个轮子的独立脉冲数）
void SendEncoderData(void)
{
    char buffer[128];
    uint32_t current_time_ms = GetSystemTime();
    
    // 发送四个轮子的独立脉冲数
    int len = snprintf(buffer, sizeof(buffer), 
                      "/four_wheel_encoder,%d,%d,%d,%d,%lu\n",
                      robot_state.enc1, robot_state.enc2, 
                      robot_state.enc3, robot_state.enc4,
                      current_time_ms);
    
    if(len > 0 && len < sizeof(buffer)) {
        for(int i = 0; i < len; i++) {
            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
            USART_SendData(USART1, buffer[i]);
        }
    }
}

// 发送IMU数据（保持不变）
void SendIMUData(void) {
    char buffer[128];
    uint32_t current_time_ms = GetSystemTime();
    
    static uint8_t error_count = 0;
    static uint32_t last_error_time = 0;
    
    // 使用局部变量避免命名冲突
    short accel_data[3], gyro_data[3], temp_data;
    
    // 修改为无下划线的函数调用
    MPU6050ReadAcc(accel_data);    // 读取加速度数据
    MPU6050ReadGyro(gyro_data);    // 读取陀螺仪数据  
    MPU6050ReadTemp(&temp_data);   // 读取温度数据
    
    // 赋值给结构体
    imu_data.accel_x = accel_data[0];
    imu_data.accel_y = accel_data[1]; 
    imu_data.accel_z = accel_data[2];
    imu_data.gyro_x = gyro_data[0];
    imu_data.gyro_y = gyro_data[1];
    imu_data.gyro_z = gyro_data[2];
    imu_data.temp = temp_data;
    
    // 检查数据有效性
    if(imu_data.accel_x == 0 && imu_data.accel_y == 0 && imu_data.accel_z == 0) {
        error_count++;
        uint32_t current_time = GetSystemTime();
        
        if(current_time - last_error_time > 5000) {
            printf("IMU数据读取失败 (错误计数: %d)，请检查MPU6050连接\r\n", error_count);
            last_error_time = current_time;
        }
        return;
    }
    
    error_count = 0;
    
    int len = snprintf(buffer, sizeof(buffer), 
                      "/imu_data,%d,%d,%d,%d,%d,%d,%d,%lu\n",
                      imu_data.accel_x, imu_data.accel_y, imu_data.accel_z,
                      imu_data.gyro_x, imu_data.gyro_y, imu_data.gyro_z,
                      imu_data.temp, current_time_ms);
    
    if(len > 0 && len < sizeof(buffer)) {
        for(int i = 0; i < len; i++) {
            while(USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
            USART_SendData(USART1, buffer[i]);
        }
    }
}

// 串口初始化函数（保持不变）
void USART1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    USART_Cmd(USART1, ENABLE);
}

// 时间同步函数（保持不变）
void SyncSystemTime(uint32_t ros_time_ms)
{
    system_time_offset = ros_time_ms - (times * 10);
    time_sync_enabled = 1;
    printf("时间同步完成: ROS时间=%lums, STM32时间=%lums, 偏移量=%lums\r\n", 
           ros_time_ms, times*10, system_time_offset);
}

// 串口中断服务函数（保持不变）
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        char received_char = USART_ReceiveData(USART1);
        
        if(received_char == 'W' || received_char == 'A' || 
           received_char == 'S' || received_char == 'D')
        {
            current_cmd = received_char;
            last_cmd_time = GetSystemTime();
            printf("CMD_RECV: %c\r\n", received_char);
        }
        else if(received_char == 'T') {
            char time_buffer[12];
            int i = 0;
            uint32_t ros_time = 0;
            
            while(i < 11) {
                if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) {
                    char digit = USART_ReceiveData(USART1);
                    if(digit == '\n' || digit == '\r') break;
                    time_buffer[i++] = digit;
                }
            }
            time_buffer[i] = '\0';
            
            if(i > 0) {
                ros_time = atol(time_buffer);
                SyncSystemTime(ros_time);
            }
        }
        
        USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

int main(void)
{	
    bsp_init();
    USART1_Init();
    TIM3_Init();
    IIC_Motor_Init();
    MPU6050_Init();
    
    printf("ROS2四轮差速传感器数据发布系统初始化完成...\r\n");
    printf("数据发布格式:\r\n");
    printf("- /four_wheel_encoder,enc1,enc2,enc3,enc4,时间戳\r\n");
    printf("- /imu_data,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,temp,时间戳\r\n");
    
    // 在main函数初始化后添加诊断信息
    printf("开始硬件诊断...\r\n");

    // 检查MPU6050
    if(MPU6050ReadID() == 0) {
        printf("MPU6050初始化失败，请检查连接\r\n");
    } else {
        printf("MPU6050初始化成功\r\n");
    }

    // 检查编码器
    int test_encoder[4];
    ReadEncoderData(test_encoder);
    printf("编码器测试读数: %d,%d,%d,%d\r\n", 
           test_encoder[0], test_encoder[1], test_encoder[2], test_encoder[3]);
    
    #if MOTOR_TYPE == 1
    Set_motor_type(1);
    delay_ms(100);
    Set_Pluse_Phase(30);
    delay_ms(100);
    Set_Pluse_line(11);
    delay_ms(100);
    Set_Wheel_dis(67.00);
    delay_ms(100);
    Set_motor_deadzone(1900);
    delay_ms(100);
    #endif

    control_pwm(0, 0, 0, 0);
    delay_ms(1000);
    
    printf("系统就绪，开始发布传感器数据...\r\n");
    
    last_calc_time = GetSystemTime();
    last_send_time = GetSystemTime();
    last_imu_time = GetSystemTime();

    while(1)
    {
        ExecuteCommand(current_cmd);
        CheckCommandTimeout();
        
        uint32_t current_time = GetSystemTime();
        
        // 如果刚停止，延迟一段时间再采集数据以减少抖动影响
        static uint32_t last_stop_time = 0;
        if(current_cmd == 0 && (current_time - last_stop_time < 50)) {
            // 停止后的50ms内不采集数据，给电机一点时间完全停稳
        } else {
            // 正常数据采集
            if(current_time - last_send_time > SEND_INTERVAL) {
                CalculateOdometry();
                SendEncoderData();
                last_send_time = current_time;
            }
            
            if(current_time - last_imu_time > IMU_INTERVAL) {
                SendIMUData();
                last_imu_time = current_time;
            }
        }
        
        if(upload_counter >= 200) {
            // 简化系统状态显示，只显示四个轮子的脉冲数
            printf("系统状态 - 四轮脉冲: %d,%d,%d,%d | IMU: 正常采集\r\n", 
                   robot_state.enc1, robot_state.enc2, robot_state.enc3, robot_state.enc4);
            upload_counter = 0;
        }
        
        times++;
        upload_counter++;
    }
}

// 替换原有的 GetSystemTime 函数
uint32_t GetSystemTime(void)
{
    uint32_t current_time;
    
    // 确保原子操作
    __disable_irq();
    current_time = monotonic_time_ms;
    __enable_irq();
    
    // 确保时间戳单调递增
    if(current_time <= last_timestamp) {
        current_time = last_timestamp + 1;
    }
    last_timestamp = current_time;
    
    return current_time;
}

// 平滑控制函数
void SmoothControlSpeed(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    const int16_t max_acceleration = 100; // 最大加速度限制
    int16_t target_speeds[4] = {m1, m2, m3, m4};
    
    // 平滑过渡
    for(int i = 0; i < 4; i++) {
        int16_t diff = target_speeds[i] - last_motor_speeds[i];
        
        // 限制加速度
        if(abs(diff) > max_acceleration) {
            target_speeds[i] = last_motor_speeds[i] + 
                (diff > 0 ? max_acceleration : -max_acceleration);
        }
        
        last_motor_speeds[i] = target_speeds[i];
    }
    
    control_speed(target_speeds[0], target_speeds[1], 
                  target_speeds[2], target_speeds[3]);
}




void ExecuteCommand(char cmd)
{
    int speed = 300;
    static uint32_t last_stop_time = 0;

    switch(cmd)
    {
        case 'W':
            SmoothControlSpeed(speed, speed, speed, speed);
            break;
        case 'S':
            SmoothControlSpeed(-speed, -speed, -speed, -speed);
            break;
        case 'A':
            SmoothControlSpeed(-speed, -speed, speed, speed); 
            break;
        case 'D':
            SmoothControlSpeed(speed, speed, -speed, -speed); 
            break;
        default:
            // =========== 修改开始 ===========
            
            // 1. 发送 PWM = 0。这是"开环"停止，相当于切断电机动力 / 空挡滑行。
            // 相比 SmoothControlSpeed(0...) 的"闭环"停止，这不会触发 PID 震荡。
            control_pwm(0, 0, 0, 0); 
            
            // 2. 重置平滑控制的速度记忆。
            // 这样下次按下 W 键时，速度会从 0 慢慢升上去，而不是突变。
            last_motor_speeds[0] = 0;
            last_motor_speeds[1] = 0;
            last_motor_speeds[2] = 0;
            last_motor_speeds[3] = 0;

            // =========== 修改结束 ===========
            
            last_stop_time = GetSystemTime();
            break;
    }
}

void CheckCommandTimeout(void)
{
    uint32_t current_time = GetSystemTime();
    if((current_time - last_cmd_time) > CMD_TIMEOUT && current_cmd != 0)
    {
        current_cmd = 0;
    }
}