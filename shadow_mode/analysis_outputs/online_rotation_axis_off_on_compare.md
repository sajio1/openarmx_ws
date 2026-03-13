# 在线旋转分段测试（OFF vs ON）

说明：按主运动窗口三等分作为 `segment1~3`（对应你 case5 的连续旋转动作阶段）。

| segment | dominant_axis(OFF/ON) | target_span_max OFF(m) | target_span_max ON(m) | ON/OFF 比值 | joint_step_max OFF | joint_step_max ON |
|---|---|---:|---:|---:|---:|---:|
| 1 | roll / roll | 0.0213 | 0.0254 | 1.19 | 0.0779 | 0.0794 |
| 2 | roll / roll | 0.0439 | 0.0441 | 1.00 | 0.0788 | 0.0618 |
| 3 | pitch / pitch | 0.0406 | 0.0374 | 0.92 | 0.0762 | 0.0794 |

## 判定
- OFF: PASS (max target span=0.0439m)
- ON: PASS (max target span=0.0441m)
- 两种模式 joint_step_max 均 < 0.1，未见爆冲。