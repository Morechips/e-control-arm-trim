# JY61 接入与第一版车身航向保持

> 历史测试记录。此报告中的 `HEADING_CORRECTION_SIGN=+1` 和旧速度/协议参数已被实车结果与后续固件修改取代；当前配置以 `Core/Inc/heading_config.h`、`Core/Inc/turn_config.h` 和 `README.md` 为准。

VERDICT: 软件实现与模拟回归完成，正常/航向低速测试/原麦轮测试三套固件 Build PASS。没有烧录或实测纠偏方向。

## 接收与硬件配置

- JY61 VCC→5V、GND共地，RX←PD5 USART2_TX，TX→PD6 USART2_RX，115200 8N1。
- USART2 RX：DMA1 Stream5 Channel4、CIRCULAR、256B。TIM6 从原100ms调整为5ms，仅提交新增字节到软件暂存区，主循环负责解析和控制。原512B/100ms配置由本次需求取代。
- 115200 8N1 下5ms最多约58字节，低于256B；DMA不是协议帧边界。中断内没有 PD、日志格式化或 Reset Z。
- 解析固定11B，搜索55和51/52/53，SUM校验前10字节；支持错位、碎片、多帧、噪声、坏帧后重新同步。
- little-endian int16，51按16g、52按2000°/s、53按180°量程换算。Yaw与GyroZ独立时间戳；只收到51不能维持航向数据有效。
- 采集时间取5ms接收服务的时间，不把积压数据的“解析时刻”冒充采集时刻。任一Yaw/GyroZ超过100ms失效；旧批次和传输错误会使解析状态失效。接收服务带来最多约一个5ms周期的时间量化。
- `JY61_Data` 提供 ax/ay/az、gx/gy/gz、roll/pitch/yaw、last_update_ms、valid，并额外记录Yaw/Gyro时间戳、帧序号及非稳定Gyro事件序号。

接口：`JY61_Init()`、`JY61_Process()`、`JY61_IsValid()`、`JY61_GetYaw()`、`JY61_GetGyroZ()`、`JY61_GetData()`、`JY61_ResetHeading()`。

RAW_YAW / GYRO_Z 的 AVAILABLE 表示软件解析字段和接口可用、模拟数据已验证，不表示本轮已从实物读取到数值。

## 归零与状态机

唯一归零字节：

```text
FF AA 01 04 00
```

`JY61_ResetHeading()` 在主循环以HAL UART TX发送5字节，TX超时上限5ms。不会停止RX DMA，不修改接收到的Yaw数值。成功发送后只丢弃归零前的软件接收积压/半帧，并使旧Yaw失去确认资格。DMA继续接收，必须由新的完整有效53帧确认。

```text
MANUAL_TURN → WAIT_STABLE → RESET_Z → WAIT_RESET_CONFIRM
                                        ↓ 新Yaw绝对值≤2°
                                       HOLD ⇄ CORRECTING
                                        ↓ 数据超时
                                       FAULT
```

- 旋转输入绝对值>1轮速等效RPM：自动保持OFF，直接使用用户/程序旋转指令，重置本次归零尝试标志；不发送Reset Z。
- 回到旋转死区后：连续有效Gyro采样中 |GyroZ|<2°/s 持续100ms，才发送一次归零。重复读取同一个样本不能累计满100ms；同一批次中出现过高Gyro，也会打断稳定计时。
- `RESET_Z` 是发送期间的瞬时状态，5Hz日志通常观察到WAIT_RESET_CONFIRM。
- 新53帧满足|Yaw|≤2°，且Gyro/Yaw新鲜，才设置RESET_Z_CONFIRMED、HEADING_HOLD。
- 300ms仍未确认则锁存RESET_FAILED、保持OFF，不自动重发；TX失败也锁存。下一次明确主动转向或显式重新参考请求才允许再试。
- 已建立过参考后IMU掉线：立即OFF、纠偏归零、FAULT。新鲜数据恢复时，重新等待稳定并以一次恢复事件重新归零确认；超时/TX失败的锁存不会被掉线恢复绕过。
- 上电不主动发送归零，没有参考时保持OFF。参考建立由一次主动转向结束，或明确调用 `Heading_RequestReference()` 触发。该调用必须在安全门已打开、IMU新鲜、没有主动转向且不处于归零过程时进行。
- 刹车/失能/失联/等待归中期间参考请求被拒绝，保持关闭；刹车解锁后需要新的明确参考事件或主动转向事件，不凭旧参考自动旋转。

## PD 与麦轮集成

参数集中在 `Core/Inc/heading_config.h`：

| 参数 | 当前值 |
|---|---:|
| HEADING_START_DEG | 5° |
| HEADING_STOP_DEG | 2° |
| HEADING_GZ_STABLE | 2°/s，严格小于 |
| HEADING_STABLE_MS | 100ms |
| JY61_TIMEOUT_MS | 100ms，超过后失效 |
| JY61_RESET_TIMEOUT_MS | 300ms |
| JY61_RESET_CONFIRM_DEG | 2°，小于等于 |
| HEADING_KP | 1.0 RPM/° |
| HEADING_KD | 0.2 RPM/(°/s) |
| OMEGA_CORR_MAX | 10轮速等效RPM |
| HEADING_CORRECTION_SIGN | +1，实车确认后必要时只改此处 |
| HEADING_LOG_PERIOD_MS | 200ms（5Hz） |

保持时|Yaw|≥5°进入纠偏，进入后只有|Yaw|≤2°才退出，2°至5°之间保留纠偏状态。

```c
error = wrap_to_180(-yaw);
omega_correction = HEADING_CORRECTION_SIGN * (Kp * error - Kd * gyro_z);
// 限制在 -10..+10 RPM
mecanum_drive(vx_user, vy_user, omega_final);
```

没有主动旋转时omega_user=0；有主动旋转时omega_correction=0，用户优先。vx/vy继续进入麦轮接口，不先停车再回正。误差按硬件Yaw相对0°计算，不通过减去软件零点伪造JY61归零。

正常模式保持原二维蓝牙摇杆：Y→vx、X→vy，没有添加旋转UI。独立程序输入为 `Car_Control_SetRotationCommand(omega_rpm)`，调用前必须已启用且通过原安全门，至少每200ms刷新；输入失效或显式0结束主动转向。

航向控制在Emm42之外实现。SHA256逐文件对照确认 `motor_driver.c`、`mecanum.c`、`emm42_driver.c` 与本次开始前完全一致：编号、地址、motor_sign、麦轮公式、协议字节均保留。

## 刹车和失效

原有brake_lock规则保留：按下刹车优先四轮FE，松开但摇杆未收到新的中心死区包时不解锁。航向安全门关闭时vx/vy日志为0，omega_user/final/correction为0，禁止归零及自动转向。

IMU超时期间用户平移/主动旋转仍可由有效人工指令控制，但绝不使用旧Yaw自动旋转。当自动纠偏期间掉线，先用原停止抢占机制取消未发出的速度/同步队列，再以omega=0恢复允许的平移。串口上最多允许原来一个在途帧完整结束；不宣称机械停止无延迟。

原PD10独立测试和原麦轮方向测试模式不启用航向自动控制，避免改变它们的既有测试动作；JY61仍可接收。

## USART1 日志

U1 PA9/PA10 改为 **115200 8N1**，停用原USART2↔U1原始透传，由JY61独占USART2接收。否则原9600带宽无法完整输出所需5Hz诊断。

每200ms输出RAW_YAW、GYRO_Z、HEADING_STATE、HEADING_HOLD、RESET_Z_SENT、RESET_Z_CONFIRMED、YAW_ERROR、OMEGA_CORRECTION、VX、VY、OMEGA_USER、OMEGA_FINAL，另含IMU_VALID、FAULT原因。Yaw单位°、Gyro单位°/s；VX/VY/OMEGA为轮速等效RPM。日志是控制请求，不冒充编码器实测速度。

归零事件单独输出request、TX字节、yaw_before、confirmed/yaw_after或timeout/TX failed。无逐字节日志。队列拥塞时跳过周期日志，日志不阻塞安全控制。

FAULT数字：0=无参考，1=IMU过期，2=归零失败，3=安全门禁止，4=无故障。初次尚未转向或建立参考，FAULT且HOLD=0属于设计行为。

## 低速航向实车测试镜像

推荐首次烧录：`build/heading-test/stm32f407_bt_oled.hex`，调试用同目录`.elf`。

```powershell
cmake -S . -B build/heading-test -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCAR_HEADING_TEST_MODE=ON
cmake --build build/heading-test --parallel 4
```

此配置使用现有蓝牙进行低速平移，限制|vx|+|vy|≤30轮速等效RPM；叠加纠偏最大10，轮速目标不超过40 RPM。调试器主动转向固定20 RPM、1秒。正常固件仍保留原平移速度范围，首次方向验证请用本低速镜像。

调试器实时变量（CPU保持运行）先写`heading_test_action`，再把`heading_test_request`加1；启动值0不会自动执行。

| action | 明确请求 |
|---:|---|
| 0 | 四轮停止并进入brake_lock |
| 1 | 顺时针20 RPM转1秒，然后旋转指令回零 |
| 2 | 逆时针20 RPM转1秒，然后旋转指令回零 |
| 3 | 显式请求建立/重试参考，仍需等待稳定和新53确认 |

必须蓝牙在线、已经启用且归中解锁；被安全门拒绝的请求会被消费，不会等解锁后补执行。正常固件不执行该测试邮箱。上位程序可以使用 `Car_Control_SetRotationCommand()` 和 `Heading_RequestReference()`，但不能绕过刹车状态机直接驱动电机。

例如：action=1，然后request=1；下一次action=2，再将request=2。测试时不要暂停运行中的CPU，软件限时与安全状态机需要继续运行。原蓝牙“立即刹车”始终有效。

## NEXT_REAL_CAR_TEST

1. 静止：检查RAW_YAW有数据、GYRO_Z接近0、IMU_VALID=1。没有明确参考事件前保持OFF。
2. 未建立保持时手动转动车身：观察Yaw与GyroZ变化，确认不是把角速度当作角度。
3. 蓝牙在线归中解锁后，触发action=1或2：确认MANUAL_TURN，自动纠偏为0，不发Reset Z。
4. 1秒旋转命令结束：确认WAIT_STABLE，低Gyro持续100ms后仅一次发送 **FF AA 01 04 00**。
5. 检查新的53帧Yaw回到±2°，出现confirmed及HOLD；若超过300ms则timeout、HOLD=0，不继续自动重发。
6. 静止保持时轻微使Yaw偏到+5°以上，观察是否向0°纠偏；首次只用当前±10RPM限幅。
7. 同样验证Yaw<-5°时的方向。每项确认车身回到±2°停止纠偏。
8. 低速前进/横移时造成角度偏差，确认平移持续且叠加反向omega，回到目标后只取消omega。
9. 验证刹车未归中锁定、IMU断线、复连归零确认、归零失败不重发。

**如果+Yaw偏差后车身继续向+Yaw旋转，立即刹车停止。只核对/反转 `HEADING_CORRECTION_SIGN`，不要修改motor_sign、麦轮公式或M1~M4编号。**

## 验证与最终字段

- 正常ARM GCC：Flash 24504B、RAM 11824B。
- 低速航向ARM GCC：Flash 24980B、RAM 11840B。
- 原麦轮测试ARM GCC：Flash 25232B、RAM 11824B。
- 三套均启用-Wall -Wextra -Werror并生成ELF/HEX/BIN。
- Keil工程已登记新增源码、XML校验通过；本轮未执行Keil编译。
- 主机回归：原整车/PD10/麦轮测试/桥接测试、JY61解帧、归零命令、防重发、新帧确认、超时锁存、隐藏Gyro高值、迟滞/PD/限幅、数据过期/复连、tick回绕、平移+纠偏实际组帧、刹车优先、低速测试邮箱全部通过。
- 模拟验证不包含真实JY61采样、设备归零响应、UART波形、车身惯性或纠偏方向。

```text
VERDICT: 软件实现及模拟测试完成；实车待验证
JY61_USART2: PASS (software)
USART2_CONFIG: 115200 8N1
JY61_DMA: PASS (configuration/build)
DMA_MODE: CIRCULAR
DMA_BUFFER: 256
FRAME_51: PASS
FRAME_52: PASS
FRAME_53: PASS
CHECKSUM: PASS
RAW_YAW: AVAILABLE (parser/API; no live reading this run)
GYRO_Z: AVAILABLE (parser/API; no live reading this run)
RESET_Z_COMMAND: FF AA 01 04 00
RESET_Z_IMPLEMENTATION: PASS
RESET_Z_REPEAT_PROTECTION: PASS
RESET_Z_CONFIRMATION: PASS (new-frame logic simulation)
HEADING_STATE_MACHINE: PASS
START_THRESHOLD: 5 deg
STOP_THRESHOLD: 2 deg
TURN_END_STABLE: |GyroZ| < 2 deg/s for 100 ms
IMU_TIMEOUT: 100 ms
BRAKE_LOCK_PRESERVED: PASS
MECANUM_INTEGRATION: PASS
BUILD: PASS
HARDWARE_DIRECTION: UNCONFIRMED
```

FILES_CHANGED:

- 新增 `Core/Inc/jy61.h`、`Core/Src/jy61.c`。
- 新增 `Core/Inc/heading_config.h`、`Core/Inc/heading_control.h`、`Core/Src/heading_control.c`。
- 修改 `Core/Inc/usart2_dma.h`、`Core/Src/usart2_dma.c`：256B/5ms、接收时间戳、不断DMA的软件屏障。
- 修改 `Core/Inc/car_control.h`、`Core/Src/car_control.c`：安全门、独立旋转命令、低速测试邮箱、平移叠加omega。
- 修改 `Core/Inc/main.h`、`Core/Inc/car_config.h`、`Core/Src/main.c`：U1日志、取消透传、JY61主循环。
- 修改 `Core/Inc/serial_io.h`、`Core/Src/serial_io.c`：周期日志的非阻塞容量检查。
- 修改 `CMakeLists.txt`、`MDK-ARM/stm32f407_bt_oled.uvprojx`。
- 修改 `tests/car/run.ps1`、`tests/car/test_car.c`、`tests/car/stm32f4xx_hal.h`。
- 新增 `tests/car/mock_jy61_transport.c`、`tests/car/test_heading.c`、`tests/car/heading_cases.inc`。
- 更新 `README.md`，新增本报告 `JY61_HEADING_REPORT.md`。
