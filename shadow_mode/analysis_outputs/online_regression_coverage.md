# 在线回归覆盖矩阵（当前）

| case | mode | pass | R_span(m) | L_span(m) | R_step_max | L_step_max | clutch_edges |
|---|---|---:|---:|---:|---:|---:|---:|
| case1_static | OFF | PASS | 0.0009 | 0.0009 | 0.0794 | 0.0794 | 0 |
| case2_right_xyz | OFF | FAIL | 0.4494 | 0.0308 | 0.0729 | 0.0559 | 0 |
| case3_left_xyz | OFF | PASS | 0.0159 | 0.2136 | 0.0683 | 0.0710 | 0 |
| case6_bimanual | OFF | PASS | 0.2261 | 0.2203 | 0.0681 | 0.0685 | 1 |
| case6_bimanual | ON | PASS | 0.2261 | 0.2203 | 0.0794 | 0.0767 | 0 |

## 已有专项结果
- case4 clutch 边沿: PASS
- case5 右手旋转 OFF: PASS
- case5 右手旋转 ON : PASS（以分段窗口判定为准，见 `online_rotation_axis_off_on_compare.md`）
- case5 左手旋转 OFF: PASS
- case5 左手旋转 ON : PASS（见 `online_rotation_left_off_on_compare.md`）

## 仍未覆盖
- 无（并发仲裁已修复并通过复测，见 `online_remaining_tests_report.md`）