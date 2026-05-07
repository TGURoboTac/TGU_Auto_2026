## TRobot

**T**GU **R**obot: The new robot embedded development framework.

# RoboTac TGU
![Build Status](https://img.shields.io/badge/Build-CLion-blue)
![Platform](https://img.shields.io/badge/Platform-STM32F7-brightgreen)


## 项目简介

> 本项目为 ***RoboTac 自动车*** 控制代码，采用 *达妙mc02开发板* 与 *gyj电机驱动板* 开发

`2026.05.07` 配合图传和遥控器，增加手动控制模式

## 软件架构

### 核心任务模块 (`app/task/`)

系统通过 FreeRTOS 进行多任务调度，各任务独立运行： 

其中，自动车的逻辑通过具体的外部信号实现的切换不同的状态


| 任务文件             | 功能描述           |
|:-----------------|:---------------|
| **`chassis.cc`** | 循迹、底盘、升降、夹爪主逻辑 |
| **`gimbal.cc`**  | 升降与夹爪的驱动       |
| **`def.h`**      | 声明一些状态         |

### 文件结构

```text
trobot_f4_gimbal/
├── app/                      # 应用层
│   ├── main.cc               # 初始化
│   └── task/                 # 任务模块
│       ├── chassis.cc        # 执行主逻辑
│       ├── gimbal.cc         # 升降与夹爪
│       └── def.h             # 状态声明
├── bsp/                      # 板级支持包
└──components/                # 组件
