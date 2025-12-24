

# stm32-robot-base

### 🤖 移动机器人底层控制固件 (Mobile Robot Chassis Firmware)

本项目是本科毕业设计《基于激光雷达点云数据的地图构建》的底层驱动部分 。代码运行于 STM32 微控制器上，负责处理移动机器人的运动控制、传感器数据采集以及与上位机（ROS）的通信。

## 🛠️ 硬件环境 (Hardware)

* 
**主控芯片**：STM32F103RCT6 


* **传感器**：
* MPU6050 6轴惯性测量单元 (IMU) 


* 直流电机编码器 (AB相) 




* 
**上位机搭配**：香橙派 5 Plus (Orange Pi 5 Plus) 


* 
**雷达搭配**：思岚 RPLIDAR S2 



## ✨ 主要功能 (Key Features)

1. 
**运动控制**：实现了增量式 PID 算法，对直流电机进行速度闭环控制，实现差速驱动 。


2. 
**里程计解算 (Odometry)**：读取编码器脉冲，融合 MPU6050 的航向角数据，计算机器人当前的位姿 (x, y, theta) 。


3. 
**ROS 通信**：设计了自定义串口协议（Header + Data + CRC校验），实现与 ROS 上位机的双向实时数据传输（上传 Odom/IMU 数据，接收 `cmd_vel` 控制指令）。



## 📂 项目工具 (Utilities)

为了方便开发与代码管理，本项目包含以下辅助脚本：

* **`keilkilll.bat`**
* **功能**：一键清理 Keil/MDK 编译产生的中间文件（如 `.o`, `.crf`, `.dep` 等），减小项目体积，方便 Git 提交。
* **用法**：双击运行即可。


* **`python.py`**
* **功能**：递归遍历项目中的所有 `.c` 和 `.h` 源文件，并将代码内容汇总导出一个文本文件。
* **用途**：用于代码备份、统计或提交软著/论文材料。



## 🚀 快速开始 (Getting Started)

1. 克隆仓库：
```bash
git clone https://github.com/DLDLDL13579/stm32-robot-base.git

```


2. 使用 Keil uVision 打开工程文件 (`.uvprojx`)。
3. 编译并下载固件至 STM32F103RCT6。
4. 连接串口至上位机，启动 ROS 节点进行调试。

