# 2026_quadrotor

`2026_quadrotor` 是一套基于 STM32F407 的四旋翼云台/发射控制固件。工程由 STM32CubeMX 生成的底层 BSP、GSRL 通用库、云台执行机构控制、裁判系统 UI、USB 视觉通信和 FreeRTOS 任务组成。

当前可执行目标名为 `2026_quadrotor`，生成的固件位于 `build/<Preset>/2026_quadrotor.elf`。

## 目录结构

```text
.
├── Chariot/
│   ├── inc/
│   │   ├── crt_gimbal.hpp       # 云台类、模式、发射状态机接口
│   │   ├── para_gimbal.hpp      # PID、限位、发射、遥控灵敏度等调参入口
│   │   ├── UI.hpp               # 裁判系统客户端 UI 协议封装
│   │   └── drv_ws2812.hpp       # WS2812 灯带驱动接口
│   └── src/
│       ├── crt_gimbal.cpp       # 云台主控制逻辑
│       ├── UI.cpp               # UI 帧构造、分步发送、周期重初始化
│       └── drv_ws2812.cpp       # TIM1 + DMA 灯带输出
├── Task/
│   ├── inc/tsk_isr.hpp          # 中断回调声明
│   └── src/
│       ├── tsk_gimbal.cpp       # 电机/IMU/云台对象实例化，1 ms 云台任务
│       ├── tsk_imu.cpp          # 1 ms IMU 更新任务
│       └── tsk_isr.cpp          # CAN/UART 回调转发
├── CubeMX_BSP/                  # CubeMX 生成的 HAL、FreeRTOS、USB、外设初始化
├── GSRL/                        # 电机、遥控、裁判系统、算法等通用库
├── CMakeLists.txt               # 顶层构建入口
├── CMakePresets.json            # Debug/Release Ninja 预设
└── flash.cfg                    # OpenOCD + ST-Link 烧录脚本
```

## 环境要求

建议使用以下工具链：

- CMake 3.22 或更新版本
- Ninja
- ARM GNU Toolchain，提供 `arm-none-eabi-gcc`
- OpenOCD，用于 ST-Link 烧录
- VS Code + CMake Tools，可直接使用仓库内 `.vscode` 配置

首次拉取仓库后需要同步子模块：

```powershell
git submodule update --init --recursive
```

## 构建

Debug 构建：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

或直接构建目标：

```powershell
cmake --build build/Debug --target 2026_quadrotor -j 8
```

Release 构建：

```powershell
cmake --preset Release
cmake --build --preset Release
```

构建产物：

```text
build/Debug/2026_quadrotor.elf
build/Debug/2026_quadrotor.map
```

## 烧录

`flash.cfg` 默认使用 ST-Link 和 STM32F4 OpenOCD target：

```powershell
openocd -f flash.cfg
```

默认烧录文件为：

```text
build/Debug/2026_quadrotor.elf
```

如果改用 Release 固件，需要同步修改 `flash.cfg` 中的 `program` 路径。

## 运行时任务

主要运行链路如下：

1. `gimbal_task()` 初始化 USB、CAN1、DR16 串口，并调用 `gimbal.init()`。
2. `Gimbal::init()` 初始化 CAN1/CAN2、DR16、VT13、裁判系统 UART1、UI、IMU 和 WS2812。
3. `gimbal_task()` 每 1 ms 调用一次 `gimbal.controlLoop()`。
4. `imu_task()` 每 1 ms 调用一次 `gimbal.imuLoop()` 更新姿态。
5. 中断回调由 `Task/src/tsk_isr.cpp` 转发到 `Gimbal` 对象。

`Gimbal::controlLoop()` 的执行顺序是：

```text
modeSelect()
targetOrientationPlan()
shootPlan()
pitchControl()
yawControl()
shootControl()
ledControl()
transmitGimbalMotorData()
transmitGimbalDataViaUsb()
UI::process()
```

## 云台控制

云台模式定义在 `Chariot/inc/crt_gimbal.hpp`：

- `GIMBAL_NO_FORCE`：无力，云台电机和发射机构停止输出。
- `CALIBRATION`：校准模式，目前保留入口。
- `MANUAL_CONTROL`：手动控制。
- `AUTO_CONTROL`：视觉自瞄控制。

模式选择入口在 `Chariot/src/crt_gimbal.cpp::modeSelect()`：

- DR16 右拨杆下：无力。
- DR16 右拨杆中：手动。
- DR16 右拨杆上：自瞄。
- VT13 可用且 DR16 未接入，默认从无力进入手动。
- VT13 鼠标右键按住时进入自瞄，否则保持手动。
- 仅 VT13 在线时，VT13 模式拨杆上档请求无力。

手动云台角度规划在 `targetOrientationPlan()`：

- DR16 右摇杆控制 yaw/pitch。
- VT13 鼠标位移叠加到 yaw/pitch 输入。
- 灵敏度、死区和鼠标等效摇杆增益在 `Chariot/inc/para_gimbal.hpp` 中调整。

自瞄模式下，USB 收到 `rxMsgViaUsb.found != 0` 时，直接使用视觉发送的 `yaw` 和 `pitch` 作为目标角。

## 发射机构

发射链路分成两层：

- `shootPlan()`：根据 DR16/VT13 输入、摩擦轮状态和裁判系统热量生成发射请求。
- `shootControl()`：执行拨弹状态机、摩擦轮闭环和堵转/反转状态。

当前状态机：

- `stateIdle`：空闲，等待单发或连发触发。
- `stateFeeding`：拨弹正转，到达目标圈数并且速度足够低后回到空闲。
- `stateReversing`：手动反转退弹，后续发射请求可直接打断反转并继续下一发。

主要操作：

- DR16 左拨杆从中到上：切换摩擦轮开关。
- DR16 滚轮离开中位：触发一次单发。
- DR16 左拨杆下：连发。
- VT13 `X`：打开摩擦轮。
- VT13 `Ctrl + X`：关闭摩擦轮。
- VT13 鼠标左键按下：触发一次单发；按住时按连发节拍持续发射；松开后停止连发请求。
- VT13 触发键：触发一次单发。
- VT13 `Z`：手动反转退弹。
- VT13 `Ctrl + Q/E`：摩擦轮目标转速分别增加/减少 `10.0f`。

热量保护使用裁判系统 17 mm 热量数据：

- 当前热量：`g_referee.getPowerHeatData().shooter17mmBarrelHeat`
- 热量上限：`g_referee.getRobotStatus().shooterBarrelHeatLimit`
- 代码为 17 mm 发射保留 2 发热量余量，每发按 10 热量计算。

发射调参入口在 `Chariot/inc/para_gimbal.hpp`：

- `FRICTION_TARGET_ANGULAR_VELOCITY`
- `FRICTION_KP/KI/KD`
- `RAMMER_INNER_*`
- `RAMMER_OUTER_*`
- `FEED_STEP_REV`
- `FEED_REV_EPS`
- `FEED_SPEED_EPS`
- `CONT_FIRE_PERIOD_MS`

## 电机与外设

电机对象实例化在 `Task/src/tsk_gimbal.cpp`：

| 机构 | 电机 | 主要总线/说明 |
| --- | --- | --- |
| Yaw | GM6020，ID 3 | CAN1 |
| Pitch | DM4310，ID 1 | CAN2 控制发送 |
| 拨弹 | M2006，ID 6，减速比 36 | CAN1 |
| 左摩擦轮 | M3508，ID 4 | CAN1 |
| 右摩擦轮 | M3508，ID 1 | CAN1 |
| IMU | BMI088 | SPI1，Mahony 姿态解算 |
| 灯带 | WS2812，120 颗 | TIM1 CH1 + DMA2 Stream5 |

电机发送在 `Gimbal::transmitGimbalMotorData()`：

- CAN1 发送 yaw + 拨弹控制帧。
- CAN2 发送 pitch 控制帧。
- CAN1 发送左右摩擦轮控制帧。

CAN1/CAN2 接收回调都会转发到 `receiveGimbalMotorDataFromISR()`。

## 通信接口

| 接口 | 配置 | 用途 |
| --- | --- | --- |
| CAN1 | CubeMX 配置，普通模式 | GM6020、M2006、M3508 控制和反馈 |
| CAN2 | CubeMX 配置，普通模式 | DM4310 pitch 控制和反馈 |
| USART1 | 115200，8N1，TX/RX | 裁判系统链路和客户端 UI |
| USART3 | 100000，9E1，RX | DR16 遥控器 |
| USART6 | 921600，8N1，TX/RX | VT13 图传遥控 |
| USB CDC | Full Speed CDC | 上位机/视觉数据收发 |

UART 引脚和 DMA 以 `CubeMX_BSP/Src/usart.c` 为准。当前代码中：

- USART1：PB7 RX，PA9 TX；RX DMA2 Stream2，TX DMA2 Stream7。
- USART3：PC11 RX，PC10 TX。
- USART6：PG9 RX，PG14 TX；RX DMA2 Stream1，TX DMA2 Stream6。

## USB 视觉协议

USB CDC 接收结构定义在 `CubeMX_BSP/Inc/usbd_cdc_if.h`：

```c
typedef struct {
    uint8_t header;              // 0xA3
    float pitch;
    float yaw;
    uint8_t found;
    uint8_t shootOrNot;
    uint8_t singleShootModeFlag;
    uint8_t checksum;
} __attribute__((packed)) rxMsgViaUsb_t;
```

接收入口在 `CubeMX_BSP/Src/usbd_cdc_if.c::CDC_Receive_FS()`：

- 仅处理 `header == 0xA3` 的帧。
- 长度必须等于 `sizeof(rxMsgViaUsb_t)`。
- 通过 `memcpy` 更新全局 `rxMsgViaUsb`。

USB CDC 发送结构：

```c
typedef struct {
    uint8_t header;              // 0x3A
    float roll;
    float pitch;
    float yaw;
    float q[4];
    float bulletSpeed;
    uint8_t toogleTargetKeyPressed;
    uint8_t EOF_;                // 0xAA
} __attribute__((packed)) txMsgViaUsb_t;
```

发送入口在 `Gimbal::transmitGimbalDataViaUsb()`，当前发送欧拉角、四元数、占位弹速和目标切换按键状态。

## 裁判系统 UI

UI 逻辑在 `Chariot/src/UI.cpp` 和 `Chariot/inc/UI.hpp`：

- 通过 USART1 发送客户端 UI 帧。
- 通过真实机器人 ID 计算客户端 ID。
- `UI::process()` 每次只发送一个阶段，避免连续 DMA 发送导致 `HAL_BUSY`。
- 静态 UI 会周期重初始化，默认间隔 `10000 ms`。
- UI 最小发送间隔 `100 ms`。

当前 UI 内容包括：

- 摩擦轮状态文字：`friction: off/on`
- 退弹/卡弹提示：`jamming`
- 辅助路线线段

UI 只有在裁判系统连接并且机器人 ID 有效时才会处理：

```cpp
if (g_referee.isConnected() && m_uiInterface->setRobotID(g_referee.getRobotID())) {
    m_uiInterface->process(HAL_GetTick(), m_frictionState, m_shootState == stateReversing);
}
```

如果客户端 UI 为空，优先检查：

1. USART1 是否确实接入裁判系统。
2. `g_referee.isConnected()` 是否为真。
3. 机器人 ID 是否有效，客户端 ID 是否能被正确计算。
4. UI 发送是否被 `HAL_BUSY` 或发送节拍挡住。

## WS2812 灯带

灯带驱动在 `Chariot/src/drv_ws2812.cpp`：

- LED 数量：`WS2812_LED_NUM = 120`
- 定时器：`TIM1`
- 通道：`TIM_CHANNEL_1`
- DMA：`DMA2_Stream5`

颜色逻辑在 `Gimbal::ledControl()`：

- 红方机器人 ID 显示红色。
- 蓝方机器人 ID 显示蓝色。
- 无有效 ID 时沿用上一次颜色，初始化默认红色。

## 调参入口

大部分常用参数集中在 `Chariot/inc/para_gimbal.hpp`：

| 类型 | 关键宏 |
| --- | --- |
| Yaw PID | `YAW_OUTER_*`、`YAW_INNER_*` |
| Pitch PID | `PITCH_OUTER_*`、`PITCH_INNER_*` |
| 摩擦轮 PID | `FRICTION_*` |
| 拨弹 PID | `RAMMER_INNER_*`、`RAMMER_OUTER_*` |
| IMU | `MAHONY_*`、`GYRO_OFFSET_*`、`ACCEL_OFFSET_*`、`INSTALL_SPIN_MATRIX` |
| 遥控输入 | `DT7_STICK_*`、`DT7_MOUSE_*` |
| 云台限位 | `PITCH_UPPER_LIMIT`、`PITCH_LOWER_LIMIT`、`YAW_MOTOR_ENCODER_*` |
| 发射 | `FRICTION_TARGET_ANGULAR_VELOCITY`、`FEED_*`、`CONT_FIRE_PERIOD_MS` |

修改控制参数后建议至少执行：

```powershell
cmake --build build/Debug --target 2026_quadrotor -j 8
```

## 开发注意事项

- `CubeMX_BSP/` 是 CubeMX 生成区，重新生成后需要复查 UART、DMA、CAN、USB 和 FreeRTOS 任务是否仍与代码匹配。
- `GSRL/Dependence/eigen` 和 `GSRL/Dependence/CMSIS-DSP` 是子模块，CI 和本地构建都依赖 `--recursive` 拉取。
- 发射逻辑同时受输入、摩擦轮状态和裁判系统热量限制影响，排查时需要同时看 `shootPlan()` 和 `shootControl()`。
- PITCH 主闭环使用 IMU pitch 和 gyro 反馈；pitch 电机角度当前主要用于可选重力补偿路径。
- YAW 目标使用 IMU yaw，机械硬限位检查使用 yaw 电机编码角。
- USB 接收目前只检查帧头和长度，`checksum` 字段尚未参与校验。
- UI 是否显示不只取决于帧内容，还取决于裁判系统连接、机器人 ID、客户端 ID 和 UART1 收发链路。
