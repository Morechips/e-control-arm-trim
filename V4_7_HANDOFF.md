# v4.7统一控制版开发交接

交接日期：2026-10-04。目标仓库 `Morechips/e-control-arm-trim`，分支 `feature/merged-team-arm-20261003`。本版从旧合并提交 `765a35c252ce1521ae8fee0b93067056b2d8c2ad` 继续开发，队友整车基线为 `e7404c592df92c7c5e7a9d09562e302ebf52e2b3`。构建标识仍为 `v4.7-unified-20261003`，不是v4.8录入/动作组测试版。

## 1. 接手人首先要知道的结论

机器人包含底盘和机械臂两个功能边界。车辆仍由Car_Control及电机、转向、视觉模块控制；人工机械臂的所有入口统一经过ArmTrimInput与ArmTrimService。传输和舵机协议继续复用原Servo/UART模块。

机械臂有三个平面关节000～002与夹爪003。固定姿态不包含003。机械臂参考、固定姿态、微调、夹爪共享同一个服务及Servo执行令牌，服务串行执行。夹爪独立不等于可以与臂杆并行动作。

微调是相对前后运动：保持参考抓口高度与方向，X正远离车，负靠近车。长按wt的数字字段是正负方向；精确毫米位移使用MoveRelativeX或文本DX。车辆必须满足输入层的停车条件。

当前没有真实舵机到位或抓取成功反馈；命令接受、UART发完、预计完成、物理到位及任务成功是不同事件。完整自主任务与视觉机械臂闭环还未接入。

## 2. 仓库与目录辨别

|对象|用途|
|---|---|
|GitHub feature/merged-team-arm-20261003|本次v4.7统一服务发布分支|
|GitHub main / releases/v4.6|原v4.6单机械臂发布历史，不随本次分支推送变化|
|本机 e-control-unified-20261003|开发、实测用v4.7工程；工作区改动从765a35c继续实现|
|本机 e-control-merged-publish-20261003|本次整理文档、复跑测试和推送的发布副本|
|本机 e-control-unified-recorder-20261003 / e-control-arm-recorder-20261003|后续v4.8录入工具及多步动作组测试副本，未混入本次发布|
|releases/merged-team-20261003|旧合并版本证据，仅供追溯|
|releases/v4.7-unified-20261003|本次构建证据；源码克隆后须重新构建镜像|

先读AGENTS.md、README和UNIFIED_CONTROL_GUIDE，再读服务头文件和本交接。不要根据manifest的旧文件名判为v4.6，也不要复制历史硬编码RAM地址做诊断。

## 3. 接入链路与各层职责

手机二进制帧或文本先由bluetooth_driver接收、校验和解析。底盘字段由PublishControl转换为CarRemoteInput_t提交车模块。晚段Bluetooth_DispatchServoActions计算机械臂姿态/GAP按下沿，把请求交给ArmTrimInput_SubmitCombined。输入层处理停车、链路、冲突和长按，再调用统一服务。服务持有Servo令牌，通过已有队列向舵机控制板发ASCII。

|层|文件|接手时应放入这层的变化|
|---|---|---|
|数学/规划核心|arm_kinematics、arm_collision、arm_trim|正逆解、轨迹、连续分支及几何约束；禁止依赖HAL/蓝牙/底盘|
|本车安装配置|arm_trim_project、arm_trim_project_config.h、arm_collision_config.h|杆长、末端偏移、标定、关节限位、障碍物/包络|
|执行服务|arm_trim_service.h/.c|拥有令牌，固定姿态/夹爪/微调、状态/等待/取消收尾|
|输入适配|arm_trim_input.h/.c|手动遥控策略、停车/链路互锁、按下沿、长按续租、回复|
|蓝牙协议适配|bluetooth_driver.h/.c|字节布局、合法性、底盘中立快照与薄分发|
|协议与传输|servo、uart_tx_queue、uart_driver|众灵协议、发送队列、实际STARTED/TC与错误处理|
|任务/视觉层（待接）|mission_fsm、route_fsm及未来适配器|操作模式、任务顺序、视觉有效期、服务完成/失败反馈|

不要覆盖HAL的全局接收/完成回调；物理串口统一留在uart_driver。生产构建使用generic core＋project＋service＋input；arm_control、arm_tuner及legacy_project留作历史主机测试，不是当前人工臂执行链。

## 4. 必须保留的主循环顺序

main.c当前前台循环先处理JY61、蓝牙、MaxiCam和板级输入，然后：

1. Car_Control_Process先做车辆安全处理。
2. Motor_Process推进真实电机发送。
3. 再次Car_Control_Process在同一循环处理电机反馈/传输故障。
4. PID_Tuner_Process。
5. ARM_TRIM_ENABLE=1时，Bluetooth_DispatchServoActions先提交请求，ArmTrimInput_Process先仲裁互锁再推进服务，服务内Servo_Process推进发送。
6. Debug/Board显示等后续处理。

ARM_TRIM_ENABLE=0保留原Servo_Process与旧蓝牙分发路径，供旧行为回归。完整构建默认开启统一服务。禁止把Servo_Process提到互锁检查之前导致失效输入下仍发出待发臂运动；也不要新增一个独立服务所有者争用Servo。

## 5. 服务接口和调用条件

所有函数加前缀 `ArmTrimService_`，见Core/Inc/arm_trim_service.h。前台调用、持续非阻塞Process；服务本身不读取摄像头、蓝牙或底盘，外层负责操作模式与停车策略。

|接口|行为|注意事项|
|---|---|---|
|Init(config, now, user)|复制配置并绑定时钟|时钟及user存储需有效，持有令牌时不能重新初始化|
|RunPreset(ServoCode)|发送固定姿态000～002|先校验目标；仅长按jog允许减速转一个待切姿态|
|ReadyProfile(profile, synchronize)|BALL/HOSTAGE/BUCKET参考姿态|true完成后同步；false只执行并释放|
|Begin(p[3], parked_stable)|由给定P建立参考|不发运动、不读P；调用者明确确认实际位置稳定|
|MoveRelativeX(dx_mm)|相对上次预计完成位置的有限位移|累计偏移以初始参考计算，越界整次拒绝|
|StartJog(direction)|向±1方向连续规划|不是每次固定2mm；边界不自动扩大|
|ReleaseJog()|正常减速停止|保留有效参考，区别于Cancel|
|Grip(pwm)|只发003|不要求参考，正常完成保留原参考；忙碌拒绝|
|Cancel()|取消待切/未发轨迹，请求停止|参考失效；不截断正在发送的UART帧|
|End()|退出释放执行权|忙碌时先取消，继续Process收尾后才释放|
|ClearFault()|已停止并释放后清故障|不恢复参考，必须重新同步|
|GetStatus()|状态、核心、传输、参考与待切|last_request不是独立完成结果|
|IsBusy()/OwnsMotion()|忙碌/令牌拥有者|有效参考静止时owns=true、busy=false，仍可接受服务内新动作|

TB_M→BALL，TH_C/TH_PRE→HOSTAGE，BD_U→BUCKET：来自安装配置。其他固定姿态从servo.c历史表读取前三项。保留ServoCode编号0～12以及原表第四项供关闭微调服务的旧路径使用；统一服务发固定姿态时始终只有三通道。

## 6. 切换、忙碌和停止细节

- 长按jog的MOVING/SETTLING、参考有效且没有待切时，新固定姿态可触发ReleaseJog并缓存一个姿态。等核心预计完成/READY后再发送。不是UART帧中途切换。
- 目标越界先拒绝，不先停止当前正常jog。已有待切时再请求返回BUSY；不是任意动作队列。
- 固定姿态、夹爪、有限DX或STOP收尾中拒绝新运动，消耗本次按下沿，空闲后不补发。
- 切姿态后持续wt=1不算新按下，必须松开再按。改方向也先松开、等待减速，再改正负号。
- STATUS可查询，STOP优先；STOP、停车条件失效及明确输入失效清待切并撤销参考。
- 只有正常wt松手/ReleaseJog保留参考。Cancel/全局STOP/故障不能描述成保留参考的暂停。
- 停止后继续周期Process，等待在途帧、逐通道停止和传输收尾；不要调用Cancel后马上停主循环。

## 7. 完成状态与故障判断

服务状态值保留：IDLE0、PROFILE1、REFERENCE2、MOTION3、GRIP4、STOPPING5、FAULT6、FIXED7。v4.7没有GROUP8或RunGroup。

参考姿态/固定姿态的运动时间为2000ms，附300ms等待；夹爪1500ms加300ms。等待从真实UART TC计时，不从“提交到队列”计时。真实传输完成只证明字节发完，尚未确认物理到位。

普通固定姿态结束尝试同步参考；模型不接受则IDLE、REF=0，可请求其他姿态。三组专用参考同步失败沿用PROFILE故障策略，进入FAULT；处理停止/传输后CLEAR并重新建立参考。两者不能混为同一种失败行为。

last_request可被后续BUSY覆盖，IsBusy=false也可能对应取消或故障。未来任务层必须记录已接受操作及预期状态，检查state、transport_result、core/reference_valid及stop_failed，不把空闲直接当成功。本版尚无稳定独立执行编号。

常见无反应定位：

|现象|先检查|
|---|---|
|所有按钮无反应|镜像核对、手机41字节类型/顺序/校验及接收计数|
|PARKED_IDLE_REQUIRED|车状态、Motor_IsIdle和Motor_HasFault，试验模式开关|
|固定姿态BUSY|动作是否还在T+guard等待，是否仍未释放按钮|
|姿态可动但wt不动|REF、STATE、方向字段、模型/启用范围与ERR|
|只有一个方向能动|MODEL_MM/ENABLED_MM及累计偏移，不只看某个舵机P|
|夹紧后再按姿态会松爪|核对是否真烧本版、是否ARM_TRIM_ENABLE=1、外部是否绕过服务发送003|
|故障后一直拒绝|等待停止完成，检查STOP_FAIL/传输，CLEAR后重建参考|

## 8. 机构参数、动作重录和配置来源

000轴心为原点，X正远离车、Z正向上，平面按Y=0。当前参数：

|项目|数值|
|---|---|
|000 / 001 / 002 P范围|915～1800 / 821～2500 / 500～1874|
|003范围、夹、松|500～2500、500、1800|
|BALL|1356、1850、698|
|HOSTAGE（待重录）|1684、2136、785|
|BUCKET|1566、1896、673|
|L1 / L2|104.85 / 84.75mm；历史口述L2=87.5mm待核对|
|末端偏移X/Z|121.1538 / 52.4mm|
|速度/加速度/更新|10mm/s / 20mm/s² / 50ms|
|最大段/最终等待/搜索窗|2mm / 300ms / ±75mm|
|后箱X/Y/Z|[-70,-30] / [-50,50] / [-31.2,38.8]mm|
|包络半径/间隙|15、20、60mm估计值；附加5mm|

参考搜索得到的连续可达区间再与启用范围交集；不自动截断越界请求，不自动换逆解分支。碰撞箱/包络为估计，不能宣称已覆盖支架、线材和开合夹爪全部风险。固定动作目标合法不表示完整路径可安全运动，也不保证结束后能同步微调参考。

重录HOSTAGE只需000～002，003保持单独控制。改参考时更新arm_trim_project_config.h或通过当前CLI配置导出/应用；同步配置JSON示例避免旧值覆盖。改其他固定姿态读servo.c前三项，保留ServoCode语义。v4.7 CLI导出的任意组数据不代表MCU有组播放器；不能直接把v4.8 schema2生成表覆盖到本版。

## 9. 后续视觉与无遥控自主任务的接入顺序

1. 定义手动/自主模式、抢占规则、操作来源及链路失效行为。服务可独立于遥控调用，但当前输入层仍有人工停车/失效互锁，不能一边自主执行一边被未仲裁的人工输入取消。
2. 任务层停车并确认稳定后，通过RunPreset/ReadyProfile伸出到参考，跟踪预计完成与REF。模型参考失败就不继续DX。
3. 腕部相机固定于夹爪组件；对各参考姿态标定视觉误差到车体X毫米。当前受限任务可先做局部一维前后偏差，不需要先承诺通用3D抓取。
4. 视觉观测必须有目标类别、时间戳、有效性和单位，处理遮挡/夹爪入画、失检与限幅。MoveRelativeX相对预计位置，不把像素直接当毫米。
5. 服务执行DX后继续观察或结束微调，再Grip。现场另记录物理到位与抓取/放桶成功，不把TC或预计完成直接作为任务验收。
6. 将mission_fsm.c中的Servo_Start任务事件适配到统一服务，并正确返回完成、失败和取消；当前该函数只设置事件/等待标志，正常main尚未启动完整Route/Mission调度。

左右误差保留ArmTrimIO_t.lateral_report / ArmTrim_ReportLateral接口，当前服务回调NULL，不自动调用底盘纠偏。要启用时需任务层协调车与臂，避免底盘动作触发现有臂取消。

## 10. 测试、构建与实机交接

本次发布在e-control-merged-publish-20261003复跑11组固件主机套件及40项Python测试，并完整ARM GCC构建；结果和哈希见V4_7_RELEASE_REPORT。以下是复跑入口：

```powershell
$env:CAR_TEST_BUILD_DIR = 'build-local'
powershell -ExecutionPolicy Bypass -File tests/arm_unified/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_trim_service/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_trim_input/run.ps1
powershell -ExecutionPolicy Bypass -File tests/arm_trim/run.ps1
powershell -ExecutionPolicy Bypass -File tests/servo/run.ps1
powershell -ExecutionPolicy Bypass -File tests/uart/run.ps1
powershell -ExecutionPolicy Bypass -File tests/car/run.ps1
python -m unittest discover -s tests/arm_setup -v
powershell -ExecutionPolicy Bypass -File scripts/build_firmware.ps1
```

其他本轮已复跑套件为arm_collision、zlis2、vision、route。fixture使用真实蓝牙/输入/服务/Servo/UART模块，替代HAL/车状态；软件PASS不能代替台架验收。

实机建议按顺序记录：

|项目|验收标准|状态|
|---|---|---|
|统一页|底盘/转向/PID及机械臂按钮各自正确|待实机验收|
|夹爪保持|夹紧后切RST/AIM/其他固定姿态不改变003|待实机验收|
|参考动作|BALL/BUCKET可同步；重录HOSTAGE后核对REF|待实机验收|
|长按微调|正负方向、松手、边界与停止符合预期|待实机验收|
|姿态切换|微调先减速切姿态，固定忙碌拒绝无补发|待实机验收|
|互锁|STOP/掉线/车开始运动取消及收尾正确|待实机验收|
|任务|近中远位置的抓球、抓人质、放桶成功及间隙记录|待实机验收|

原稳定目录与三版本烧录脚本保留，可以回退比较；本次发布没有操作真实COM、烧录或复位MCU。早期单臂长按用户已实测基本可用，不能作为本次统一版的全任务实测结果。
