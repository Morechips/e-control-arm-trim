# ZL-IS2 UART 驱动交付（2026-09-16）

## 结果与接线

用户指定 U3 后，将 **USART3、PB10 TX / PB11 RX、AF7、115200 8N1、无流控** 分配给 ZL-IS2。
接线：PB10 → ZL-IS2 RX，PB11 ← ZL-IS2 TX，双方 GND 相连。
这里的 U3 指 USART3 外设编号，不能仅凭原理图的器件位号 U3 判断接线。

本地 `D:/GDUTcom/ProPrj_工科大_2026-08-07.epro2` 的主控模块符号有 B10/B11 引出；
`D:/GDUTcom/纯插件.epro2` 也有 PB10/PB11 引出网络。当前软件未配置这两个引脚。
后者的其他 USART3 候选引脚 PC10/PC11 关联视觉接口，PD8/PD9 关联测距接口，因此本次选择 PB10/PB11。
引脚复用依据 [ST STM32F405/407 数据手册 Table 9](https://www.st.com/resource/en/datasheet/dm00037051.pdf)。
这些是软件和设计资料检查结果；未烧录、未测量实板引脚、未验证真实 ZL-IS2 应答或机械动作。

## 工程与资源检查

工程使用 STM32F4 HAL，已有可构建的 CMake/Ninja 和 Keil 项目，没有 `.ioc`、独立 `usart.c` 或 `gpio.c`。
GPIO/UART 初始化位于 `Core/Src/board_app.c`、`stm32f4xx_hal_msp.c` 及 `usart2_dma.c`。
驱动沿用 `Core/Inc` / `Core/Src` 风格，不移植厂家 F103 Demo。

| 外设 | 用途 | TX / RX | 波特率 | DMA / IRQ |
| --- | --- | --- | --- | --- |
| USART1 | 5Hz 航向诊断日志；可选桥模式 | PA9 / PA10 | 115200 | 无 DMA；USART1 IRQ 优先级 1，日志 TX IT |
| USART2 | JY61 | PD5 / PD6 | 115200 | RX DMA1 Stream5 Channel4 Circular，256B；TIM6 5ms 优先级 2；USART2 IRQ 优先级 1，无 RXNE/IDLE/DMA IRQ 接收 |
| USART3 | 本次 ZL-IS2 | PB10 / PB11 | 115200 | 阻塞 TX；无 DMA、无新增 IRQ |
| UART4 | 未启用 | 未配置 | — | 无配置 |
| UART5 | 四轮电机 | PC12 / PD2 | 115200 | TX/RX IT，IRQ 优先级 1；无 DMA |
| USART6 | 蓝牙 | PC6 / PC7 | 9600 | RX IT，IRQ 优先级 1；无 DMA |

所有已启用串口均为 8N1、无流控。OLED 仍使用 PB6/PB7、I2C1 EV/ER IRQ 优先级 3；
按钮仍使用 PD10/PD11/PD14/PD15；SysTick、主循环、原有 IRQ 回调、JY61 DMA 与电机控制逻辑均未修改。
原工程 `CAR_BOOT_AUTO_ENABLE=1` 的车轮使能行为保持；“无上电动作”在本次报告中特指没有新增 ZL-IS2 命令。

## 驱动行为与调用约束

- `Board_Init()` 中先初始化 USART3，再调用 `ZLIS2_Init(&huart3)`；初始化只校验并保存句柄，不发送命令。
- 所有 TX 统一经过 `zlis2_send()` → `HAL_UART_Transmit()`，timeout 固定 40ms；无 DMA、日志、自动重试或复位等待。
- 驱动单实例，必须串行地从前台调用，HAL tick 必须运行；不要在 ISR、关中断区域或多个并发调用者中使用。
- 15 字节单舵机帧线速约 1.31ms；24 条同步帧为 362 字节，线速约 31.43ms。上层若以后加入频繁发送，需要安排调用时机，避免阻塞现有 20ms 控制节拍。本次未向控制循环增加发送调用。
- 多舵机 buffer 为 363 字节（包含字符串结尾）；最多 24 条，这是缓冲区容量选择，不是协议 ID 上限。先检查全部参数，再构造，最后只调用一次 UART 发送。
- 每次 `snprintf` / `vsnprintf` 均检查负返回值和截断；参数/长度错误不会调用 UART。传输途中硬件故障可能已经发出部分字节，返回 `ZLIS2_UART_ERROR`，不保证物理传输原子性。
- 所有协议字符串不附加 CR/LF 或 NUL。Raw 按调用者给定内容发送，最多 362 字节；调用者负责合法协议和可读字符串。
- 舵机 ID 0–254，PWM 500–2500，时间 0–9999ms，偏差 -500–500；不换算角度，不引入机械动作含义。
- 动作/组合组编号、次数和录制周期采用 `uint16_t`，支持该 C 类型完整范围，不宣称这是控制器的协议上限。
- 动作范围和已录制动作的 repeat=0 保留无限循环语义。组合组的 repeat 不附加未经确认的含义。
- HAL_OK 只表示串口发送完成，不表示控制器执行成功。RX 引脚已配置，但本阶段不实现接收或应答解析。
- 未初始化时所有发送 API 都返回错误；无效 Init 清除原绑定，避免错误重初始化后仍使用旧 UART。
- 没有嵌入式自动测试入口，所有自动化测试均在 PC 的 mock HAL 上运行，不操作实物。

## 公开 API

所有函数返回 `ZLIS2_Status`；声明见 `Core/Inc/zlis2_driver.h`。

```c
ZLIS2_Init(UART_HandleTypeDef *huart);
ZLIS2_SetServo(uint16_t id, uint16_t pwm, uint16_t time_ms);
ZLIS2_SetServos(const ZLIS2_ServoCommand *commands, size_t count);
ZLIS2_RunAction(uint16_t action);
ZLIS2_RunActionRange(uint16_t start_action, uint16_t end_action, uint16_t repeat);
ZLIS2_StopAll(void);
ZLIS2_StopServo(uint16_t id);
ZLIS2_SetServoBias(uint16_t id, int16_t bias);
ZLIS2_RunCombinedAction(uint16_t group, uint16_t repeat);
ZLIS2_RecordPose(void);
ZLIS2_RunRecorded(uint16_t repeat);
ZLIS2_ClearRecorded(void);
ZLIS2_SetRecordPeriod(uint16_t period_ms);
ZLIS2_Reset(void);
ZLIS2_SendRaw(const char *command);
```

## 验证

协议依据为用户提供的已确认 ZL-IS2 指令，未增加未确认协议。

1. `./tests/zlis2/run.ps1`：**102 项检查 PASS**。
   覆盖全部 API 字节与长度、用户要求的例子、上下界、正负偏差、repeat=0、uint16 最大值、24 条精确容量、超量/SIZE_MAX、Raw 精确长度/越界、非法数组末项整帧不发、空指针、静默初始化/失败重绑定，以及 HAL_ERROR/BUSY/TIMEOUT 不重试。
2. `cmake --build cmake-build-debug --parallel 4`：**ARM GCC 编译链接 PASS，0 errors，0 warnings**，启用 `-Wall -Wextra -Werror`。
   正常镜像 Flash 24728B、RAM 11904B；输出 ELF/HEX/BIN。
   未调用的发送 API 会被现有 `--gc-sections` 正常裁剪，上层加入调用并重编译后会链接对应代码。
3. `./tests/car/run.ps1`：**全部原有主机回归 PASS**，含蓝牙整车、PD10、麦轮测试模式、USART2 桥、JY61/航向与航向整车模式。
4. Keil 工程已登记 `.c/.h`，XML 解析及路径检查通过；本次未执行 Keil 编译，编译验收以实际使用的 CMake/ARM GCC 为准。

## 修改清单

新增：`Core/Inc/zlis2_driver.h`、`Core/Src/zlis2_driver.c`、`tests/zlis2/stm32f4xx_hal.h`、
`tests/zlis2/test_zlis2.c`、`tests/zlis2/run.ps1`、本报告。

修改：`Core/Inc/main.h`、`Core/Src/board_app.c`、`Core/Src/stm32f4xx_hal_msp.c`、
`CMakeLists.txt`、`MDK-ARM/stm32f407_bt_oled.uvprojx`、`README.md`。

## 首次硬件测试建议（未执行）

只在确认 ID 0 舵机当前位于中位附近、1520 PWM 位于安全范围后，显式调用一次：

```c
ZLIS2_SetServo(0, 1520, 1000); /* #000P1520T1000! */
```

不要从整组运动开始。该示例没有加入上电路径或主循环。
