# v4.5：切换参考姿态保留夹爪

2026-10-02。修复蓝牙测试页切换抓球前、抓人质前、放球前时附带 003 目标、改变夹持状态的问题。

## 固定动作与夹爪

三个参考的配置只存 000～002；`@BENCH READY`、`@BENCH PREP` 及其 7 字节/11 字节蓝牙入口均用掩码 7，仅发送三关节。独立夹紧 P500、松开 P1800 仍只发 003。固定动作完成后自动同步三关节参考的机制不变。

| 场景 | 000 | 001 | 002 | 003 | 当前状态 |
|---|---:|---:|---:|---|---|
| BALL 抓球前 | 1356 | 1850 | 698 | 不发送 | 待验证此次夹爪修复 |
| HOSTAGE 抓人质前 | 1684 | 2136 | 785 | 不发送 | 旧安装值，待重录 |
| BUCKET 放球前 | 1566 | 1896 | 673 | 不发送 | 待验证此次夹爪修复 |

可以夹紧后切换放球前，到位并完成微调后才单独松开。重新选择固定动作仍会返回参考并清零累计微调量，因此不应在完成微调后用同一固定动作代替夹紧。

修改限于这三个独立测试参考。原团队 bool、PRESET 等其他入口可能有自己的夹爪动作，未统一删除；日常用现有 8 bool 测试页。连续微调速度、关节范围、几何、箱体保护和通信协议沿用 v4.4。

## 抓人质重录待办

用户确认机械结构改变，尚未录好新姿态。本次保留旧三关节值，不自行推算替换；HOSTAGE 暂不作为新结构的有效参考用于抓取验收。

退出微调并使用已能正常工作的上位机调整到实际抓人质前姿态，记录 000、001、002 的 P 值，重复返回确认姿态。只需提供这三个数值，003 无需录入。后续更新 `Core/Inc/arm_trim_project_config.h` 的 `ARM_TRIM_HOSTAGE_P0/P1/P2`，就位与微调同步共用同一组配置；重新检查模型范围、构建并烧录后验证。若还改变了轴心距或舵机安装零位，也须更新相应几何/标定，重录 P 值本身不能代替几何标定。

## 验证

- `tests/arm_bt/run.ps1`：29186 项主机 HAL 模拟检查通过。新增夹紧/松开后切换全部 READY/PREP 参考均无 003 指令，旧 11 字节和当前 7 字节路径均检查不含 003；既有通信、微调及互锁回归通过。
- `scripts/build_firmware.ps1`：完整 ARM GCC 编译链接通过，text 82204 B、data 108 B、bss 24724 B。
- 本轮没有连接、烧录或驱动硬件。物理保持夹持、抓取和放球验收待填写 `ARM_TRIM_ACCEPTANCE.md`。

旧 v4.4 镜像和原配置保存在 `rollback_v4_4_20261002`；当前镜像在 `firmware_direct`，哈希见 `FIRMWARE_SHA256.txt`。HEX SHA256：`1A483F1C17E29B9DCDEA57541E9A3ECCF6590727CB6803ADAF7F4D6CDDD6E852`。启动标识为 `ARM TRIM FW=V4.5 L2_MM=84.75 WT=HOLD_JOG BYTE0=0_OR_84`。

## 烧录与检查

车辆停车并确认目标、供电、SWD 接线及机械臂支撑后，由用户执行：

```powershell
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\flash_firmware.ps1" -ConfirmHardwareReady
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-trim-test-20261001\scripts\verify_firmware.ps1"
```

第一条使用脚本中的 OpenOCD 及已配置的 STM32F407/CMSIS-DAP 目标，烧写并校验后复位；`-ConfirmHardwareReady` 表示运行者已确认硬件准备。第二条独立比较当前 HEX，成功应显示 `CURRENT_FIRMWARE_VERIFIED`。

先验证夹紧后切换抓球前、放球前仍夹紧，再验证松开后切换仍松开；抓人质待重录后补测。手机页面无需修改。用户现已恢复 GitHub 上传，先保存当前软件基线；抓人质新参考与实车验收完成后再更新验收版本。
