# 8方向平移与航向保持修改报告

## 结论

本次在现有蓝牙、安全状态机、JY61、航向 PID、麦轮和电机分层上完成最小范围修改。普通摇杆控制已从连续旋转语义改为二维平移：`X` 为左右、`Y` 为前后；航向修正只由 JY61/PID 产生。指定目标角和 ±5°微调通过修改 `target_yaw` 实现，不直接给电机持续角速度。

主机回归和 ARM GCC 固件构建通过；未烧录、未驱动实车，纠偏正负方向仍需低速确认。

## 1. 修改文件

- `Core/Inc/car_config.h`：删除普通连续旋转的速度配置。
- `Core/Inc/heading_config.h`：新增 `MAX_YAW_CORRECTION_RPM=100` 和 `ANGLE_ADJUST_STEP_DEG=5`。
- `Core/Inc/mecanum.h`、`Core/Src/mecanum.c`：恢复 X 型麦轮二维平移 + 航向修正公式及统一缩放。
- `Core/Inc/heading_control.h`、`Core/Src/heading_control.c`：改为软件零点、持久 `target_yaw`、目标角微调、跨界归一化和 IMU 异常降级。
- `Core/Inc/car_control.h`、`Core/Src/car_control.c`：摇杆 X/Y 改为横移/纵移；取消连续旋转入口，增加目标角 API；保留安全门。
- `Core/Inc/mecanum_test.h`、`Core/Src/mecanum_test.c`：测试动作只保留 STOP 和 8 个平移方向。
- `tests/car/test_car.c`、`tests/car/mecanum_cases.inc`、`tests/car/heading_cases.inc`、`tests/car/test_heading.c`：更新并扩充回归场景。
- `README.md`、本报告：更新当前行为和实车测试表。

未修改 JY61 底层解析/配置、Emm42 协议、UART/DMA、OLED、电机地址和方向标定。

## 2. 修改函数

- `Mecanum_Calculate()`、`mecanum_drive()`：参数语义改为 `forward/right/heading_correction`，按四轮麦轮公式输出。
- `Bluetooth_ControlProcess()`：`c->y→vx`、`c->x→vy`，摇杆不再产生 `omega_user`。
- `Car_Control_SetTargetYaw()`、`Car_Control_AdjustTargetYaw()`：新增经过蓝牙在线、刹车、失能、故障和运行状态安全门的角度命令。
- `Heading_Init()`：默认目标为 0°，等待第一份可信数据建立软件零点。
- `Heading_Update()`：计算相对航向、±180°误差、PID修正、100 RPM限幅和故障降级。
- `Heading_RequestReference()`：把当前原始航向设为软件 0°，不发送 JY61 硬件归零命令。
- `Heading_SetTarget()`、`Heading_AdjustTarget()`：维护绝对目标和相对微调目标。
- `Mecanum_Test_Vector()`：保留 8 个标准平移方向，移除测试中的连续旋转动作。

## 3. 取消/禁用的 360°连续旋转控制

- 摇杆 X 不再映射到转向/旋转量，纯 X 输入现在是横移。
- 删除 `rotation_command`、刷新超时和 `Car_Control_SetRotationCommand()` 连续角速度入口。
- 删除麦轮测试动作 `MEC_ROTATE_CW`、`MEC_ROTATE_CCW`。
- 航向修正只能来自 PID；主动改变姿态只能设置/增减目标角。

## 4. 8方向平移实现

生产摇杆保持连续二维平移，因此也自然包含 8 个标准方向；没有对摇杆强制量化。测试邮箱提供前、后、左、右、左前、右前、左后、右后八个明确动作，斜向使用等幅 X/Y 分量，对应 45°。

## 5. 四轮平移公式

坐标为 `+vx=前`、`+vy=右`、`+omega=顺时针修正`，逻辑轮序为 M1右前、M2左前、M3左后、M4右后：

```text
M1 = vx - vy - omega
M2 = vx + vy + omega
M3 = vx - vy + omega
M4 = vx + vy - omega
```

安装方向修正仍只在 `motor_driver.c` 应用，运动学层没有重复反转。

## 6. target_yaw 维护

- 建立软件参考后默认 `target_yaw=0°`。
- 普通平移只读取目标，不在每次起步时重写为 0°。
- `Car_Control_SetTargetYaw(angle)` 设置指定目标。
- `Car_Control_AdjustTargetYaw(delta)` 在旧目标上增减；测试微调步进为 ±5°。
- 到达目标后保留最后目标，后续平移继续锁定该角度。
- 只有显式 `Heading_RequestReference()` 会重新定义软件零点并把目标置 0°。

## 7. 陀螺仪 0°定义

安全门打开后第一次收到完整且新鲜的 JY61 GyroZ/Yaw 数据时：

```text
yaw_zero = raw_yaw
current_yaw = normalize(raw_yaw - yaw_zero)
```

软件参考不会改 JY61 底层寄存器。JY61 原有硬件归零函数仍保留在底层以兼容其他用途，但本控制链不调用它。

## 8. 航向 PID 如何叠加

```text
yaw_error = normalize(target_yaw - current_yaw)
yaw_correction = clamp(PID(yaw_error, gyro_z), -100, +100)
wheels = mecanum(vx, vy, yaw_correction)
```

PID 保留原 `P + I - Kd*GyroZ` 结构、5°启动/2°停止滞环和在线调参。修正作为独立 `omega` 项进入四轮公式，不改写用户的 `vx/vy`。

若 JY61 未建立参考、帧无效或数据超过 100 ms，`yaw_correction=0`；不会用旧数据猛转，也不会因 IMU 单独失效而强制取消用户平移。

## 9. 航向修正限幅

`MAX_YAW_CORRECTION_RPM = 100.0f`。

## 10. 最高轮速

`MOTOR_MAX_RPM` 仍为 500。混控后查找四轮最大绝对值；超过 500 时四轮乘同一比例，所以任何最终逻辑目标都满足 `|Mi|<=500 RPM`，同时保留轮间比例。

## 11-12. 角度微调和主动角度变化

角度变化继续使用现有航向 PID 和同一套麦轮旋转项，不直接人为维持某个电机角速度。±5°微调和指定角度都会更新 `target_yaw`；达到目标后新目标不会被下一次平移清零。

当前正常 13 字节摇杆帧没有角度按钮字段，因此没有改变蓝牙数据格式。上层可调用 `Car_Control_AdjustTargetYaw(±ANGLE_ADJUST_STEP_DEG)` 或 `Car_Control_SetTargetYaw(angle)`；航向测试固件的 action 1/2/3 分别为 +5°、-5°和软件重新归零。

## 13. 刹车安全锁

保持原逻辑：按下 BRAKE 立即排队四轮急停并进入 `CAR_BRAKE_LOCK`；松开按钮本身不能恢复，必须收到新的 X/Y 同时位于死区的数据且电机队列空闲才解锁。回归覆盖了“刹车脉冲、松开仍锁定、归中后解锁”。

## 14. 电机方向配置

保持不变：

```text
M1 右前：-1
M2 左前：+1
M3 左后：+1
M4 右后：-1
```

## 15. 八方向四轮目标速度符号

`+/-/0` 表示逻辑轮速的正/负/零；物理符号列已经乘上固定 `motor_sign`。斜向为等幅 X/Y 输入，非零轮达到同一目标幅值。

| 方向 | M1,M2,M3,M4 逻辑符号 | M1,M2,M3,M4 物理符号 |
|---|---|---|
| 前 | `+ + + +` | `- + + -` |
| 后 | `- - - -` | `+ - - +` |
| 左 | `+ - + -` | `- - + +` |
| 右 | `- + - +` | `+ + - -` |
| 左前 | `+ 0 + 0` | `- 0 + 0` |
| 右前 | `0 + 0 +` | `0 + 0 -` |
| 左后 | `0 - 0 -` | `0 - 0 +` |
| 右后 | `- 0 - 0` | `+ 0 - 0` |

该表是当前软件模型的目标符号，不是编码器实测结果。实车应先在低 RPM、车轮悬空或可靠支撑、急停可触达的条件下逐项验证。

## 验证结果

- `powershell -ExecutionPolicy Bypass -File tests/car/run.ps1`：通过。包含正常控制、PD10独立模式、8方向测试模式、JY61/航向单元、航向整车集成、PID调参和UART桥回归。
- `cmake --build build --parallel 4`：通过，`-Wall -Wextra -Werror`；Flash 28,360 B（5.41%），RAM 13,664 B（10.42%）。
- 未执行烧录、车轮悬空测试或地面行驶测试。
