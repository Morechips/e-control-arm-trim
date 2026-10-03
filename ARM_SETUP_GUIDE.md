# 机械臂安装向导与总线舵机动作录入

2026-10-03，配置工具 v1，适配当前 v4.6 模块化工程。工具在电脑上运行，不需要先烧录新固件。它只向舵机板发送 `#NNNPRAD!` 查询，不发送移动、释力、复位、动作回放或烧录指令。

## 1. 快速启动

当前这台车，沿用现有参数作为录入起点，在 PowerShell 运行：

```powershell
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-arm-trim-latest-20261003\scripts\setup_arm.ps1" -Port COM13 -UseCurrentExample
```

新机械安装不要使用 `-UseCurrentExample`：

```powershell
powershell -ExecutionPolicy Bypass -File "D:\工科大\e-control-arm-trim-latest-20261003\scripts\setup_arm.ps1" -Port COM13
```

`COM13` 替换成舵机板 USB 或 USB 转 TTL 的实际端口。默认 115200、8N1。封装脚本优先使用本机 `D:\Anaconda\python.exe`，其他电脑寻找 `python`；可通过 `-PythonExecutable "C:\...\python.exe"` 指定 Python 3.9 或更新版本。

需要 pyserial。当前电脑已经安装；其他电脑缺少时，使用同一个 Python 安装依赖：

```powershell
python -m pip install -r scripts/arm_setup_requirements.txt
```

列出端口只枚举设备，不连接：

```powershell
python -X utf8 scripts/arm_setup.py ports
```

## 2. 向导菜单

| 选项 | 用途 |
| --- | --- |
| 1 | 机械尺寸、工具偏移和方向偏角 |
| 2 | 000/001/002 限位及两点角度标定；P可自动读取 |
| 3 | 003夹爪的范围、夹紧和松开P；可自动读取 |
| 4 | 单箱体障碍物、组件包络半径和额外间隙 |
| 5 | 微调速度、加速度、范围、周期、固定动作和夹爪时间 |
| 6 | 自动录动作点，可追加多个步骤，也可更新三个微调参考 |
| 7 | 检查配置完整性、单位和协议范围 |
| 8 | 导出配置头、动作数据表、CSV及串口指令预览 |
| 9 | 备份后把三份配置头应用到当前工程 |
| 0 | 保存退出 |

默认工作文件 `configs/arm_installation.json`。可通过 `-Config` 指定每台机械臂各自的 JSON。首次使用 `-UseCurrentExample` 才填入当前机器参数；已有文件不会被覆盖。当前示例中的 HOSTAGE 是旧安装值，碰撞包络含估计，不代表新机器已经完成标定或验收。新安装模板的机械尺寸、限位、标定和参考为空；缺项会阻止导出和应用。新安装启用障碍物模型时也必须自行测量，不会默认继承旧箱体。

每完成一个菜单操作保存一次，前一份 JSON 保留为同名 `.bak`。录入可以分几次完成，缺项允许暂存，但不能导出。Ctrl+C退出，当前尚未提交的菜单操作不保存。

## 3. 尺寸、角度和限位怎么填

- 原点为 **000轴心**；X正方向朝抓取物体/远离车；Z正方向向上；机械臂所在平面是Y=0。长度单位为mm。
- L1是000轴心到001轴心；L2是001轴心到002轴心；测转轴中心距离，不测板材外缘长度。
- `tool_x_mm/tool_z_mm` 是腕部局部坐标里，从002轴心到选定抓点的偏移。各关节角为零时各杆沿+X；工具偏移不是当前姿态下世界坐标的前后/高度距离。
- `tool_axis_offset_deg` 是夹口方向相对腕部X方向的角差。
- 000角度相对+X；001、002角度相对前一根杆。角度输入deg，导出时转为rad。每关节填两个已知角度与对应的P，两个P、两个角均需不同；可使用负斜率标定。
- 限位按P数值大小填`min_pwm/max_pwm`，允许范围500..2500。向导不会自动扫到机械极限；由操作者把关节调整到已确认的边界，才查询并保存。
- 夹爪通道固定003。当前封装只支持三个平面关节加夹爪，不支持任意关节数量或ID重映射。

P回读可以帮助录标定点，但不能自动量出臂长、安装零位、工具偏移或障碍物。关节活动限位也不等于每个姿态都无碰撞。

## 4. 自动录动作组

先用现有控制方式把机械臂调好、等待稳定。使用USB/TTL直接连接舵机控制板，STM32到舵机板的串口线断开；关闭占用同一COM的软件。选择菜单6，填写动作组英文名、平面步骤或夹爪步骤、动作时间和等待时间，再确认查询。

采集只查询所需通道，读取两整轮，要求每个通道的P差不超过2；全部成功后保存最后一轮。每次读取后关闭串口，方便继续用现有工具调下一个姿态。操作者仍需确保姿态已经稳定，这不是基于速度反馈的到位检测。

示例动作组 `take_ball`：

1. 选平面步骤，调到抓球前，读取000/001/002；同时填写参考`BALL`。
2. 选夹爪步骤，调到夹紧，读取003；同样填`take_ball`，追加第2步。
3. 选平面步骤，调到抬起，读取000/001/002；填`take_ball`，追加第3步。

**平面步骤只包含000/001/002，夹爪步骤只包含003**。三个微调参考永远只保存三个关节；录参考不会把夹爪开合值带进去。普通动作组通过独立步骤表达夹爪操作，避免切换平面姿态时意外松开。

相同组名继续追加；可以混合不同时间的步骤。时间T是计划动作时间，`guard_ms`是动作后的额外稳定等待，均由操作者填写，不能靠读取P推算。当前三个参考按钮共用`reference_move_ms`；各个自定义动作点的独立时间保存在动作组里。

更新抓人质参考的快捷命令：

```powershell
python -X utf8 scripts/arm_setup.py record --config configs/arm_installation.json --port COM13 --group hostage_reach --profile HOSTAGE --time-ms 2000 --guard-ms 300
```

只有已填写三关节限位才能录平面动作。读到越界值会拒绝，不自动裁剪P。没有返回、回包格式不匹配、错ID、值不合法、姿态变化过大或串口断开，都不会保存半个姿态或补零。错误显示有限长度原始HEX，便于定位。

录入错误时可以重新录参考覆盖原参考；动作组点可以在JSON的`action_groups[].points`里删除或调整顺序，再校验。原始采集记录不是“已验收”标记，参考默认仍待重复返回及任务验收。

查询协议依据本地官方《众灵舵机使用手册-250508》第26页：`#002PRAD!` → `#002P0785!`。用户已确认本机为总线舵机及此实际回包格式。协议没有序列号，工具清缓冲并双轮采集，但无法证明晚到的同ID回包一定属于本次查询；录入期间应保持串口独占、不同时发送其他指令。

## 5. 导出、应用与烧录

菜单8导出到默认 `build-local/arm_setup/generated`：

- `installation.json`：完整可迁移配置及采集来源。
- `arm_trim_project_config.h`：尺寸、标定角和P、三关节范围、三个参考、夹爪范围和开合P、微调参数。
- `arm_collision_config.h`：一个箱体与三个组件包络。
- `arm_trim_input_config.h`：参考/夹爪动作时间、稳定等待。
- `arm_recorded_actions.h`：有序动作组C数据表；mask=7为平面步骤，mask=8为夹爪步骤。
- `action_steps.csv`：逐点查看和交流。
- `action_commands.txt`：逐步串口命令预览，工具不会发送这些命令。
- `manifest.json`：导出文件SHA256。

菜单9或以下命令应用配置。先校验，再备份三份原头文件到 `build-local/arm_setup/backup-*`，只替换这三份配置；写入失败或Ctrl+C会恢复原配置。不会修改队友的`servo.c`固定动作表、蓝牙布局、几何核心，也不会编译或烧录。

```powershell
python -X utf8 scripts/arm_setup.py validate --config configs/arm_installation.json
python -X utf8 scripts/arm_setup.py export --config configs/arm_installation.json
python -X utf8 scripts/arm_setup.py apply --config configs/arm_installation.json
```

导出预览不会写入工程 `Core/Inc`。旧v4.5或未接入可配置标定角/夹爪限位的工程会拒绝apply，需先接入当前配置适配代码。

**应用后，三个微调参考和配置在重新构建/烧录后生效。自定义多步动作组目前是录入/导出数据，尚未绑定为蓝牙按钮或接入固件多步播放器。** 后续任务模块可读取C表逐步运行，必须等待当前步骤UART实际发完，再等待T+guard，之后才派发下一步；不能把全部点一口气提交给普通Servo的latest队列，否则待发点会被替换。播放平面固定动作的路径仍需独立验证。

确认配置和验收范围后，在本工程构建并由操作者烧录：

```powershell
Set-Location 'D:\工科大\e-control-arm-trim-latest-20261003'
$env:PATH = 'C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin;' + $env:PATH
powershell -ExecutionPolicy Bypass -File .\scripts\build_firmware.ps1
# 确认目标板、供电、SWD、机械臂支撑及车轮离地后执行：
powershell -ExecutionPolicy Bypass -File .\scripts\flash_firmware.ps1 -ConfirmHardwareReady
powershell -ExecutionPolicy Bypass -File .\scripts\verify_firmware.ps1
```

原先的其他固定动作可能超出本模块的微调范围，向导不改变这些历史动作或替它们验证路径。关节限位、标定点、模型安全范围和实物到位是不同的信息。

## 6. 软件验证与当前固件

配置工具测试使用假串口，不打开真实COM：

```powershell
python -X utf8 -m unittest discover -s tests/arm_setup -p "test_*.py"
powershell -ExecutionPolicy Bypass -File tests/arm_trim_service/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_trim_input/run.ps1
```

生成的配置头有真实C编译/运行测试，检查新角度、尺寸、速度和动作mask确实进入模块；服务回归检查夹爪配置范围生效、拒绝越界时不发送。

本次整合的完整固件已重新构建到本目录 `firmware_direct`；旧目录镜像未改。电脑录入脚本不依赖烧入这份新固件。尚未使用真实COM运行本次完整向导；首次录入先检查屏幕回读是否与现有工具一致。
