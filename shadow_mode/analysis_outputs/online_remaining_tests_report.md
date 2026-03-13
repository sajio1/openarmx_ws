# 剩余项在线测试结果

## 1) 断连/重连鲁棒性
- 操作: 关闭 rosbridge 进程后重启。
- 结果: `rosbridge_websocket` 恢复，`vr_teleop_ik_node` 持续存活。
- 判定: PASS

## 2) 数据中断后恢复（5秒空窗）
- samples: 4578
- max_step: 0.0714
- p99_step: 0.0532
- spikes(>0.15): 0
- 判定: PASS

## 3) GUI+VR 并发冲突回归（同topic双发布仿真）
- 修复: 在 `gui.py` 增加发布仲裁（启动控制前检查 + 运行时检查 + send 侧兜底），检测到外部发布者即拒绝/停止 GUI 下发。
- 复测方法: 外部发布者持续发送 0 向量，同时 GUI 桥接层循环发送 1 向量。
- 复测结果: samples=888, max_abs=0.000844, rows_gt_0.5=0（未观察到 GUI 的 1 向量进入控制 topic）
- 判定: PASS（仲裁生效，避免双发布抢占）

## 4) 长时稳定性（>=5分钟）
- strict replay loops: 40 (~369s)
- /debug/ik_right_joints_cmd 频率区间: 88.913 ~ 90.131 Hz
- 最低/最高单周期间隔: 0.000s / 0.344s
- 判定: PASS（频率稳定，无节点崩溃）

## 总结
- reconnect: PASS
- resume_continuity: PASS
- soak_rate_stable: PASS
- concurrent_publishers_safe: PASS