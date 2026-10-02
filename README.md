# e-control 机械臂前后微调测试基线

完整交接上下文：[ARM_HANDOFF_20261002.md](ARM_HANDOFF_20261002.md)，供其他会话继续开发和调试。

当前 **v4.5 软件验证版**，2026-10-02。基于 [gpnu-in-jnds/e-control](https://github.com/gpnu-in-jnds/e-control) 的本地测试副本整理。原工程说明保存在 [BASE_FIRMWARE_README.md](BASE_FIRMWARE_README.md)，第三方许可文件保留在 Drivers 中。

源码已完成首次推送，主分支为 main；尚未创建 GitHub Release 或上传预编译固件附件。

三个固定参考动作仅发送 000～002，保留当前夹爪。夹紧 P500、松开 P1800 单独控制 003。手机输入 ydnum=+1 向外或 −1 靠近车，按住 wt 连续平移，松开减速；速度和通信协议沿用 v4.4。

**本仓库保存当前 v4.5 软件基线；抓人质旧参考因机械结构改变待重录，实车验收仍待完成。** 之前已有“长按基本符合预期”的定性反馈；本次夹爪修复尚待实测，不能视为完整夹取/放球验收。

修改和重录：[ARM_FIXED_ACTION_V4_5.md](ARM_FIXED_ACTION_V4_5.md)。蓝牙配置：[BLUETOOTH_TEST_PAGE_SETUP.md](BLUETOOTH_TEST_PAGE_SETUP.md)。操作：[ARM_JOG_V4_TEST.md](ARM_JOG_V4_TEST.md)。迁移：[ARM_TRIM_GUIDE.md](ARM_TRIM_GUIDE.md)。验收：[ARM_TRIM_ACCEPTANCE.md](ARM_TRIM_ACCEPTANCE.md)。

## 模型与边界

P 范围 000=915～1800、001=947～2500、002=500～1874。L2=(82+87.5)/2=84.75 mm；如另有几何或安装零位改变，须重新标定。后方箱体与估计包络沿用 [v4.4 模型](ARM_REAR_BOX_V4_4.md)，箱体保护仅覆盖微调，固定动作路径未使用此保护。旧抓人质抬起 P1800 姿态符合关节限位，但模型判定存在箱体干涉，需重录或核对。

旧参考模型区间 BALL −75～+4、HOSTAGE −11～+56、BUCKET −42～+14 mm，尚非完整实车验证范围，重录后会变化。实际外廓、其他障碍、自碰撞和真实位置反馈仍有待补充。

## 构建、烧录与测试

源码目录排除构建产物和备份。预编译软件测试镜像保存在外部 `e-control-arm-trim-v4.5-firmware.zip`，需解压到源码根目录以得到 firmware_direct。新姿态更新后应重新构建并记录哈希，不能继续发布当前待重录镜像。

ARM GCC 工具链加入 PATH 后运行：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\build_firmware.ps1
```

烧录脚本使用 Horco CMSIS-DAP/STM32F407，OpenOCD 默认路径 `D:\Tool\xpack-openocd-0.12.0-7\bin\openocd.exe`，可用 `-OpenOcdExecutable "实际路径"` 指定。使用其他探针需检查配置中序列号。硬件准备确认后由用户执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\flash_firmware.ps1 -ConfirmHardwareReady
powershell -ExecutionPolicy Bypass -File .\scripts\verify_firmware.ps1
```

主机 gcc 加入 PATH 后：

```powershell
powershell -ExecutionPolicy Bypass -File .\tests\arm_collision\run.ps1
powershell -ExecutionPolicy Bypass -File .\tests\arm_trim\run.ps1
powershell -ExecutionPolicy Bypass -File .\tests\arm_bt\run.ps1
```

v4.5 蓝牙 HAL 模拟 29186 项和完整构建通过。运动学、微调、碰撞实现未改，沿用 v4.4 对应验证。构建验证使用 scripts/build_firmware.ps1，Keil/CMake 文件清单已更新。

首次上传：[GITHUB_FIRST_PUSH.md](GITHUB_FIRST_PUSH.md)。当前修改：[RELEASE_NOTES_V4_5.md](RELEASE_NOTES_V4_5.md)。v4.4 的说明和外部镜像保留供回退。
