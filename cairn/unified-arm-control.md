---
type: project_topic
status: active
summary: "v4.7统一机械臂服务；用户确认001下限改821放行AIM固定姿态，保留底盘入口。"
tags: [arm, bluetooth, ownership, firmware]
contains: [decision, lesson, open_question]
created: "2026-10-03"
updated: "2026-10-04"
related: [../UNIFIED_MODULE_INTEGRATION.md, ../UNIFIED_CONTROL_GUIDE.md]
authoring_mode: ai_generated
---
# 统一机械臂服务

## 已确认的决策

用户确认所有固定姿态只动000～002，003夹爪始终单独请求。微调中请求固定姿态可减速切换；固定动作忙碌时拒绝、不重播。沿用队友41字节格式，增加未使用字段，保留底盘快照入口。

用户随后明确要求001软件下限由947改为821，以执行现有AIM（1058、821、554）。当前限位为000=915～1800、001=821～2500、002=500～1874；AIM仅动000～002，003保持。固定动作可发送不代表能微调，需检查动作完成并同步后的REF。

所有人工机械臂动作进入ArmTrimService，静止参考保留Servo令牌供微调使用。固定姿态、夹爪和微调共享此令牌，无需先END再切页面。数学核心不依赖输入或HAL。

## 已解决的陷阱

上一版服务持有令牌时调用不带令牌的Servo_SendPreset会收到BUSY。解决是统一请求入口，绕过令牌会破坏预计位置与互锁。

扩大解码结构后，短包必须清零短格式不存在的字段，否则41字节WT=1会残留到旧中立包被续租。DecodeControl现先清空快照；回归覆盖全部旧长度清字段，以及21/31字节中立周期包让长按减速停止。

主机联动使用真实蓝牙、输入、服务、Servo/UART队列，只替代硬件边界。它证明状态/协议逻辑，不证明碰撞间隙、夹取或机械到位。

## 待验证与后续问题

- HOSTAGE仍为旧安装姿态，等待重录。
- 固定目标仅检查关节限位，尚无完整固定轨迹碰撞预检；微调使用保守包络模型。
- 运行时尚无实际关节位置和夹取成功反馈，完成状态为估计。
- Servo_Start当前只是任务事件占位；自主视觉需接服务执行、完成/失败通知并定义人工/自主模式仲裁。
- 任意多步动作组播放器尚未接入；臂杆与夹爪仍按当前规则串行执行。
- 本轮001下限改821后，AIM真实发送链路、服务/核心/输入/配置向导测试与完整固件构建通过；构建输入和镜像哈希匹配。本次整理到feature/merged-team-arm-20261003发布副本；仍未烧录或开展v4.7实车验收。

详细接口见 [UNIFIED_MODULE_INTEGRATION.md](../UNIFIED_MODULE_INTEGRATION.md)，页面配置见 [UNIFIED_CONTROL_GUIDE.md](../UNIFIED_CONTROL_GUIDE.md)。

## 2026-10-04发布交接

远程核对前版为765a35c，本次发布保留相同feature分支并增加v4.7统一控制改动。README聚焦当前入口，PROJECT_REFERENCE保留上一README供硬件历史追溯；差异、交接与实际复跑记录分开维护，避免把v4.8窗口/动作组功能写成v4.7已实现。发布目录11组固件、40项Python与完整构建通过；参见[V4_7_HANDOFF](../V4_7_HANDOFF.md)和[V4_7_RELEASE_REPORT](../V4_7_RELEASE_REPORT.md)。
