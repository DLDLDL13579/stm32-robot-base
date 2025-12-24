
---

# 🤖 STM32 Robot Base Chassis Firmware

> **项目背景**：本科毕业设计《基于激光雷达点云数据的地图构建》底层驱动子系统。
> 本项目运行于 **STM32F103RCT6** 微控制器，负责移动机器人的运动控制、多传感器数据融合（IMU + 里程计）以及与 ROS 2 上位机的时钟同步与通讯。

## 📖 目录 (Table of Contents)

* [硬件架构 (Hardware)](https://www.google.com/search?q=%23-%E7%A1%AC%E4%BB%B6%E6%9E%B6%E6%9E%84-hardware)
* [核心特性 (Core Features)](https://www.google.com/search?q=%23-%E6%A0%B8%E5%BF%83%E7%89%B9%E6%80%A7-core-features)
* [通信协议 (Communication Protocol)](https://www.google.com/search?q=%23-%E9%80%9A%E4%BF%A1%E5%8D%8F%E8%AE%AE-communication-protocol)
* [数据处理逻辑 (Algorithm)](https://www.google.com/search?q=%23-%E6%95%B0%E6%8D%AE%E5%A4%84%E7%90%86%E9%80%BB%E8%BE%91-algorithm)
* [项目工具 (Utilities)](https://www.google.com/search?q=%23-%E9%A1%B9%E7%9B%AE%E5%B7%A5%E5%85%B7-utilities)
* [快速开始 (Quick Start)](https://www.google.com/search?q=%23-%E5%BF%AB%E9%80%9F%E5%BC%80%E5%A7%8B-quick-start)

## 🛠️ 硬件架构 (Hardware)

* **主控芯片**: STM32F103RCT6 (ARM Cortex-M3)
* **传感器**:
* **IMU**: MPU6050 (6轴加速度计+陀螺仪，I2C接口)
* **里程计**: AB相增量式光电编码器 (4路，定时器编码器模式)


* **执行器**: 直流减速电机 x4 (四轮差速驱动)
* **上位机**: Orange Pi 5 Plus (运行 ROS 2)
* **激光雷达**: SLAMTEC RPLIDAR S2

## ✨ 核心特性 (Core Features)

本项目不仅仅是简单的驱动代码，针对 SLAM 建图需求进行了以下特定优化：

1. **🚫 智能零点漂移抑制 (Smart Deadzone Filtering)**
* 针对 SLAM 建图对里程计的高精度要求，实现了静态死区过滤。
* 在未接收到运动指令时，强制过滤 +/- 3 脉冲内的微小抖动，防止机器人在静止状态下地图发生漂移。


2. **⏳ ROS 时间同步 (Time Synchronization)**
* 实现了 STM32 与 ROS 上位机的时间戳对齐。
* 通过串口接收 ROS 时间戳，动态计算 `system_time_offset`，确保上传的 IMU 和里程计数据具有准确的 ROS 时间基准。


3. **🏎️ 平滑速度控制 (Smooth Velocity Control)**
* **闭环加速**: 实现了基于加速度限制的平滑启动，避免电机突变导致的打滑或电流过载。
* **开环急停**: 针对停止指令，直接切断 PWM 输出（空挡滑行模式），消除 PID 减速过程中的震荡。


4. **🛡️ 鲁棒的异常处理**
* **编码器**: 包含 3 次重试机制及异常值过滤，防止干扰导致的飞车。
* **IMU**: 实时监控 MPU6050 状态，包含掉线检测与自动错误计数上报。



## 📡 通信协议 (Communication Protocol)

采用 ASCII 字符串格式进行串口通信，波特率 **115200**。

### 1. 上行数据 (STM32 -> ROS)

| 数据类型 | 格式示例 | 描述 |
| --- | --- | --- |
| **四轮编码器** | `/four_wheel_encoder,e1,e2,e3,e4,ts` | `e1-e4`: 四轮独立脉冲累计值<br>

<br>`ts`: 同步后的毫秒时间戳 |
| **IMU 数据** | `/imu_data,ax,ay,az,gx,gy,gz,temp,ts` | `ax...gz`: 原始加速度/角速度<br>

<br>`temp`: 温度 |

### 2. 下行指令 (ROS -> STM32)

* **运动控制**: 单字符指令
* `W`: 前进
* `S`: 后退
* `A`: 左旋
* `D`: 右旋
* *(无指令/超时)*: 自动停止


* **时间同步**: `T<timestamp>`
* 例: `T1634567890` (用于同步系统时间)



## 🧮 数据处理逻辑 (Algorithm)

代码中包含关键的算法处理流程：

```c
// 里程计解算逻辑片段
void CalculateOdometry(void) {
    // 1. 读取32位绝对脉冲
    // 2. 差分计算
    if (!is_moving_cmd) {
        // 3. 静态死区过滤 (防止地图漂移的关键步骤)
        if(delta_pulse >= -3 && delta_pulse <= 3) delta_pulse = 0;
    }
    // 4. 更新状态
}

```

## 📂 项目工具 (Utilities)

为了优化开发体验，项目根目录提供了以下脚本：

* **`keilkilll.bat`**
* **作用**: 一键清理 Keil/MDK 编译生成的中间文件（`.o`, `.crf`, `.dep` 等）。
* **场景**: 在提交 Git 代码前运行，可显著减少仓库体积。


* **`python.py`**
* **作用**: 源代码汇总工具。
* **场景**: 递归读取所有 `.c/.h` 文件并合并为一个文本，便于申请软件著作权或论文代码附录制作。



## 🚀 快速开始 (Quick Start)

1. **克隆项目**
```bash
git clone https://github.com/DLDLDL13579/stm32-robot-base.git

```


2. **编译烧录**
* 使用 **Keil uVision 5** 打开 `.uvprojx` 工程文件。
* 点击 `Rebuild`，确保无报错。
* 使用 ST-Link/J-Link 下载固件。


3. **硬件连接与调试**
* 连接 **USART1** (PA9/TX, PA10/RX) 至上位机。
* 打开串口助手，波特率设为 **115200**。
* 复位后应看到：`ROS2四轮差速传感器数据发布系统初始化完成...`



---

**Author**: Deng Lin

**Department**: Computer Science and Technology (Software Engineering)

---
