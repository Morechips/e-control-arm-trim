# 整车与机械臂合并版构建记录

分支 `feature/merged-team-arm-20261003`，构建标识 `v4.6-team-main-20261003`。

BUILD_MANIFEST.json 来自发布目录默认完整 ARM GCC 构建，记录 259 个实际构建输入、工具链参数及该次本地产物哈希。来源、文档和使用流程见根目录 MERGED_BRANCH_README.md。二进制和临时目录不纳入 Git，克隆后需先运行 scripts/build_firmware.ps1；重新构建的调试路径等可能改变 ELF 哈希，烧录及核对应使用同一次构建的产物和清单。

发布前复跑：tests/arm_trim_input/run.ps1（266126 + 184 检查）、tests/arm_trim_service/run.ps1（2762 + 1029 检查）、Python tests/arm_setup（40 项），以及完整默认固件构建。HEX/BIN 与已交付合并版相同；链接仍有已有 nosys 未实现系统调用提示。实车抓取/放球验收待完成，本次没有连接或操作硬件。
