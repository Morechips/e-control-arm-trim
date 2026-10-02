# 转向与 JY61 接收调试

右转 90° 最高 20 RPM，左转 90° 最高 60 RPM，右转 180° 最高 40 RPM；剩余 30°时请求初始转速和 30 RPM 中的较小值，微调也采用相同上限。控制器根据朝目标方向的 `GYRO_Z` 预估刹车后 80 ms 的滑行角，最多提前 15°刹车。陀螺仪数据超过 20 ms 未更新时使用原来的 5°停转阈值。稳定后的 ±5°判断和最多三次自动微调仍然生效。

USART2 使用 256 字节循环 DMA，默认每 5 ms 由 TIM6 搬运并在主循环解析。可单独构建 2 ms 版本进行实机比较：

```powershell
cmake -S . -B build/dma2 -G Ninja -DCMAKE_BUILD_TYPE=Debug -DUSART2_RX_PERIOD_MS=2
cmake --build build/dma2 --parallel 4
```

一次转向的串口日志包括 `START`、`SLOW_APPROACH`、`APPROACH_STOP`、`STOP_DETAIL`、`SETTLED`、`CORRECT` 和接收统计。`STOP_DETAIL` 的 `a10`、`gz10`、`lead10` 分别是角度、角速度、预计滑行角的十倍；`SETTLED` 的 `a10` 和 `err10` 同样以 0.1°为单位。`RX hz` 是本次转向期间接收的平均 yaw 帧率，`yaw_gap` 是控制器观察到的新 yaw 样本之间的最大毫秒间隔，`loop_gap` 是两次转向控制调用之间的最大毫秒间隔，`age` 是 DMA 搬运时间戳到控制调用的最大毫秒数。`DMA poll` 表示 2 ms 或 5 ms 配置，`unread`、`overflow`、`error` 是本次转向期间的计数增量。时间戳无法表示字节到达 DMA 环形缓冲区之前的传感器内部延迟。

先比较默认 5 ms 固件多次 90°和 180°转向的首次 `SETTLED` 误差、最终误差和 `CORRECT` 次数。只有在 5 ms 版本没有 DMA 丢批次或错误、`loop_gap` 留有 2 ms 周期余量，并确认实机中断负载允许时，才对比 2 ms 版本；若 2 ms 版本出现接收错误或主循环变慢，保留 5 ms。验收目标是最终误差 ±5°，通常最多一次微调。80 ms 刹车响应时间是待实机日志校准的初始值。
