## 1. 项目特性

- MCU：STM32H723（工程含 `robot.ioc`）
- 电机能力：
  - 单电机控制
  - 一拖多模式
    （这俩区别我没仔细看）
- 调试输出：USB 转串口
    （要每次上电之后才能打开串口，有点麻烦不过影响也不大）
- 急停开关：
    按下断电，开启上电

## 2. 目录说明

```text
robot/
├─ App/my_fdcan/            # FDCAN 通用收发与过滤配置
├─ Core/                    # CubeMX 生成的 HAL 初始化与中断（目录下有main.c）
├─ Drivers/                 # STM32 HAL/CMSIS （应该不用管）
├─ src/                    （这整个文件夹都是高擎官方的资料，我就改了电机id，转换单位）
│  ├─ convert/              # 数据类型与单位转换
│  ├─ livelybot_fdcan/      # 电机协议封装（底层指令）
│  ├─ motor/                # 电机状态管理、端口映射、状态解析
│  ├─ motor_control/        # 单电机高级控制接口
│  ├─ motor_many/           # 一拖多控制接口
│  └─ motor_config/         # 电机配置（回零、保存参数）
├─ test/                    （这也是官方给的测试案例）
│  ├─ test_motor/           # 单电机测试入口
│  └─ test_motor_many/      # 一拖多测试入口
├─ USB_DEVICE/              # USB Device 相关 （板子有个USB接口，我当串口用的）
├─ cmake/                   # 工具链与 CubeMX 子工程配置
├─ CMakeLists.txt
└─ CMakePresets.json
```

## 4. 快速上手

1. **硬件初始化后启动 FDCAN**
   - `main()` 中初始化 FDCAN1/FDCAN2 后调用 `fdcan_filter_init()`。
    fdcan_filter_init(&hfdcan1);
    fdcan_filter_init(&hfdcan2); #这两行不能删，删了上电不转
2. **打开电机电源**
   - 使用 GPIO 使能电机供电
    HAL_GPIO_WritePin(MOTOR1_PWR_EN_GPIO_Port, MOTOR1_PWR_EN_Pin, GPIO_PIN_SET); #PC14控制电源通断
3. **调用测试接口验证链路**
   - 单电机：`test_motor_control(id)`
   - 一拖多：`test_motor_many()`
4. **主循环中处理回包（建议）**
   - 周期调用 `motor_process_state_all()` 解析 FIFO 中电机状态数据。
   （这条是高擎官方的建议，应该在一拖多模式里要用）
   - USB 状态字中的电机/CAN 诊断位：
     - `0x0100`：6 个电机反馈均在 60 ms 内更新
     - `0x0200`：检测并丢弃过异常反馈（上电后保持置位）
     - `0x0400`：FDCAN RX FIFO 发生过丢帧（上电后保持置位）
     - `0x0800`：FDCAN 发送帧入队失败（上电后保持置位）
5. **急停开关**
    PE13配置为输入,开关的常闭端一端接PE13，另一端接5V


### 6.1 电机数量与端口

在 `src/motor/motor.h` 中配置：

- `MOTOR_PORT_NUM`：CAN 通道数
- `MOTOR_MAX_NUM`：每通道电机最大数量

### 6.2 端口映射与电机型号

在 `src/motor/motor.c` 顶部区域配置：

- `motor_state_port[][]`：每个端口下各 ID 电机型号
- `port_maping[]`：`PORTx` 到 `hfdcanx` 的映射

### 6.3 一拖多数量

在 `src/motor_many/motor_many.h` 中：

- `MANY_MOTOR_SIZE` 取值范围 `(0, 30]`


