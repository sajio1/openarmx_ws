# 在线旋转测试报告 (rot_offset=ON, latest)

- mode: rotation_on_online_latest
- input_samples: 1852
- target_samples: 2406
- jcmd_samples: 3248
- clutch_edges: 2
- target_span_xyz_m: [0.0405485, 0.30740756260863566, 0.5637839496180899]
- target_span_max_m: 0.5637839496180899
- joint_span_7: [1.7256079679339247, 1.913751014064843, 3.140000000004859, 2.395978962768761, 3.0000000000039795, 1.5000000000014695, 3.0]
- joint_step_max: 0.07937253933193777

## 判定
- stream_alive: PASS
- rotation_chain_responds: PASS
- translation_not_flying: FAIL
- joint_not_explosive: PASS