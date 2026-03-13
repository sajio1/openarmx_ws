# 在线旋转 OFF/ON 对照

| 指标 | OFF | ON |
|---|---:|---:|
| input_samples | 1935 | 1890 |
| target_span_max_m | 0.0441 | 0.3109 |
| joint_span_max | 3.1401 | 3.1400 |
| joint_step_max | 0.0792 | 0.0794 |
| translation_not_flying | PASS | FAIL |
| rotation_chain_responds | PASS | PASS |

## 结论
- OFF: 旋转响应正常，位置漂移可控。
- ON: 旋转响应正常，但出现明显平移耦合（超阈值）。