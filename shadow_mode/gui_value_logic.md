Value  |  Motor_motion  |  Gui_bar

right:
j1: up | forward | ->
j2: up | right inward | ->
j3: up | clockwise | ->
j4: up | upward | ->
j5: up | clockwise | ->
j6: up | left inward | ->
j7: up | upward | ->
ee: up | open | ->

left:
j1: up | backward | ->
j2: down | left inward | <-
j3: down | counter_c | <-
j4: up | upward | ->
j5: down | counter_c | <-
j6: down | right inward | <-
j7: down | upward | <-
ee: up | open | ->


严重问题：
value | physically | gui_bar | rviz
j6_left: down | left outward | <- | right inward
j6_right: down | right_outward | <- | right_outward

j2_left: up | left_outward | -> | left_inward
j2_right:严重问题 朝外开合被限位 rviz状况位置
right j1 和 j2现在都动不了了

semantic orientation vs. float axis
j1_left:
urdf/ros2: -3.34（向前）-----------|--- 1.41（向后）
gui: 1.41(向后) ---|----------- -3.34（向前） mirror: true

j1_right:
urdf/ros2: -1.25（向后）---|----------- 3.50（向前）
gui: -1.25（向后）---|----------- 3.50（向前）mirror：false

j2_left:
urdf/ros2: -3.27（向外）-------------|- 0.13（向内）
gui: 0.13（向内）-|------------- -3.27（向外）mirror: true

j2_right:
urdf/ros2: -0.13（向内）-|------------- 3.27（向外）
gui: -0.13（向内）-|------------- 3.27（向外）mirror: false

j6_left:
urdf/ros2: -0.75 ----|---- 0.75