# 在线旋转测试报告 (rot_offset=ON)

- mode: rotation_on_online
- clutch_msgs: 3
- clutch_edges: 2
- input_samples: 1890
- target_samples: 2360
- jcmd_samples: 3002
- input_span_xyz_m: [0.081097, 0.081322, 0.08993999999999999]
- target_span_xyz_m: [0.30312122957065063, 0.2524403934471857, 0.31091749492964477]
- joint_span_7: [1.5641708943728314, 0.7355661365338297, 3.1400000000107706, 1.268993687431505, 2.999999998244919, 1.4911094510118168, 3.00000000000016]
- joint_step_max: 0.07937253933193775

## 判定
- stream_alive: PASS
- rotation_chain_responds: PASS
- translation_not_flying: FAIL
- joint_not_explosive: PASS