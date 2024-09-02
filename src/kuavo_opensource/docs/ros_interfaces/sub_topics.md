## KUAVO 订阅的话题


### walkCommand

- 话题名称：/walkCommand
- 话题类型：dynamic_biped/walkCommand
- 描述：控制机器人行走的话题。
- 消息格式

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| mode | uint8 | 模式 |
| values | float64[] | 速度 |

NOTE:

- mode: 0->PositionCommand | 1->VelocityCommand | 2->stepCommand
- values: 长度为 3，分别为 x,y 轴速度(单位：m/s)和 yaw 角速度(单位：rad/s)

<br></br>

### control_robot_hand_position

- 话题名称：/control_robot_hand_position
- 话题类型：dynamic_biped/robotHandPosition
- 描述：控制机器人手部位置
- 消息格式

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| left_hand_position | uint8[] | 左手位置 |
| right_hand_position | uint8[] | 右手位置 |

NOTE:

- hand_posistion:
  - 末端为灵巧手时，数组长度为 6，对应大拇指关节，拇指外展肌，食指关节, 中指关节，无名指关节，小指关节, 位置范围：0-100, 0 为张开，100 为闭合
  - 末端为夹爪时，数组长度为 1，范围在 0-255，0 为张开，255 为闭合

<br></br>

### kuavo_arm_traj

- 话题名称：/kuavo_arm_traj
- 话题类型：sensor_msgs/JointState
- 描述：控制机器人手臂关节运动
- 消息格式

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| name | string[] | 关节名称 |
| position | float64[] | 位置 |
| velocity | float64[] | 速度 |
| effort | float64[] | 力矩 |

NOTE:

- 只需要填充 position 字段即可，机器人手臂关节数量为 14

<br></br>

### kuavo_arm_target_poses

- 话题名称：/kuavo_arm_target_poses
- 话题类型：dynamic_biped/armTargetPoses
- 描述：控制机器人手臂在指定时间内到达目标位置
- 消息格式

float64[] times
float64[] values

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| times | float64[] | 时间 |
| values | float64[] | 位置 |

NOTE:

- times: 机器人手臂到达目标位置的时间，全局时间，不断递增，单位：s
- values: 每一次机器人手臂目标位置，长度为 14 * times.size()

<br></br>

### robot_head_motion_data

- 话题名称：/robot_head_motion_data
- 话题类型：dynamic_biped/robotHeadMotionData
- 描述：控制机器人头部运动
- 消息格式

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| joint_data | float64[] | 关节数据 |

NOTE:

- joint_data: 机器人头部关节数据，长度为 2，分别为 yaw 和 pitch, 单位：degrees
