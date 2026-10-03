# 整车与机械臂合并版分支

本分支：`feature/merged-team-arm-20261003`。

保存已交付的合并版，默认同时启用队友整车功能与机械臂模块。此分支不是机械臂专用版，也不是队友原版。

## 来源与行为

- 在用户仓库 `Morechips/e-control-arm-trim` 的 `main` 提交 `9b61a6a52b98678d26e367a2a14a2d73641c0aa4` 上创建独立分支。
- 队友整车基线：`gpnu-in-jnds/e-control` 的 `e7404c592df92c7c5e7a9d09562e302ebf52e2b3`。
- 本次生产构建输入来自本地已验证的 `e-control-arm-trim-latest-20261003`，构建标识 `v4.6-team-main-20261003`。
- 保留整车遥控、航向、转向、视觉/射靶及原舵机动作；接入独立机械臂核心、服务、输入和安装配置工具。
- 机械臂独立页面的三个参考动作只控制 000～002；003 夹爪单独控制。`wt` 按住连续前后微调，松开减速停止。
- 抓人质参考姿态仍待改装后重录；新构建的完整实车验收待执行。当前固件不读取机械臂实际位置反馈。

只推送此新分支，不向 `main` 合并，不更改已有版本标签。历史文档、后箱示意图及 `releases/v4.6/` 原样保留供追溯；它们描述旧交付版本，不代表本分支的构建参数或验收结论。

## 使用文档

- [蓝牙控制器完整指引](BLUETOOTH_CONTROLLER_GUIDE.md)：41 字节整车页面和 7 字节机械臂页面的字段、顺序、控件、操作及协议示例。
- [模块封装与接口](ARM_TRIM_INTEGRATION.md)：核心、适配层、所有权、取消和移植接口。
- [安装配置与自动动作录入](ARM_SETUP_GUIDE.md)：串口查询当前位置、生成参数与动作数据；修改后需重新构建固件。
- [整合与验证记录](LATEST_INTEGRATION_REPORT.md)：来源、接入范围、全部软件验证及待实车项。
- [验收记录模板](ARM_TRIM_ACCEPTANCE.md)：实际抓取和放球记录。

## 从新分支获取并编译

```powershell
git clone --branch feature/merged-team-arm-20261003 --single-branch https://github.com/Morechips/e-control-arm-trim.git e-control-merged
Set-Location e-control-merged
```

先将 ARM GCC 工具链的 `bin` 目录加入 `PATH`。本机示例：

```powershell
$env:PATH = 'C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin;' + $env:PATH
powershell -ExecutionPolicy Bypass -File scripts/build_firmware.ps1
```

构建输出到 `firmware_direct/`，包含 ELF、HEX、BIN、MAP 和当前输入/固件哈希清单。产物和临时文件不纳入 Git；克隆后须先构建再烧录。`releases/merged-team-20261003/BUILD_MANIFEST.json` 保存合并版交付时的构建信息，重新构建应以新生成的清单为准。

确认 STM32F407 目标板、SWD 接线和供电、机械臂支撑及车轮离地后，执行：

```powershell
powershell -ExecutionPolicy Bypass -File scripts/flash_firmware.ps1 -ConfirmHardwareReady
powershell -ExecutionPolicy Bypass -File scripts/verify_firmware.ps1
```

两脚本可通过 `-OpenOcdExecutable '你的openocd.exe绝对路径'` 指定工具位置。烧录脚本会写入、校验并复位；核对脚本只比对镜像，短暂暂停后恢复。默认整车固件上电自动使能四轮。

## 软件验证

合并版交付时已完成机械臂核心/服务/输入、Servo/UART、底盘/视觉/路线、安装配置工具及默认完整 ARM GCC 构建，详细记录见整合报告。发布前核对 259 个构建输入与该合并版清单一致，并在发布目录复跑蓝牙输入、服务/真实 Servo 联动、安装工具测试及默认完整固件构建。软件检查不代表实车验收通过；本次发布没有操作硬件。
