## KUAVO 机器人节点发布的话题

### robot_end_effector_state

- 话题名称：/robot_end_effector_state
- 话题类型：dynamic_biped/EndEffectorStateStamped
- 描述： 机器人末端执行器状态
- 消息格式
  
| 字段 | 类型 | 描述 |
| --- | --- | --- |
| header | std_msgs/Header | 消息头 |
| names | string[] | 名称 |
| positions | float64[] | 位置 |
| velocities | float64[] | 速度 |
| torques | float64[] | 扭矩 |
| voltages | float64[] | 电压 |
| status | float64[] | 状态 |

NOTE: 

- pos：假如类型为**夹爪**, 范围在 **0-255**，0 为张开，255 为闭合
- status: 假如类型为**夹爪**， 0： 未检测到物体，1: 手指在张开检测到物体，2: 手指在闭合检测到物体，3: 手指已经达到指定的位置，没有检测物体


<br></br>

### robot_head_motor_position

- 话题名称：/robot_head_motor_position
- 话题类型：dynamic_biped/robotHeadMotionData
- 描述：机器人头部关节数据
- 消息格式

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| joint_data | float64[] | 关节数据 |

NOTE:

- 需要在机器人对应的版本 `kuavo.json` 中添加头部的电机才会发布此 topic，详情请参考[文档](../base_useage.md#头部关节配置)
- joint_data: 机器人头部关节数据，长度为 2，分别为 yaw 和 pitch

<br></br>

### robot_hand_position

- 话题名称：/robot_hand_position
- 话题类型：dynamic_biped/robotHandPosition
- 描述：机器人手部位置
- 消息格式

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| header | std_msgs/Header | 消息头 |
| left_hand_position | uint8[] | 左手位置 |
| right_hand_position | uint8[] | 右手位置 |

NOTE:

- hand_posistion: 数组长度为 6，对应大拇指关节，拇指外展肌，食指关节, 中指关节，无名指关节，小指关节, 位置范围：0-100, 0 为张开，100 为闭合

<br></br>

### leju_robot_phase

- 话题名称：/leju_robot_phase
- 话题类型：dynamic_biped/robotPhase
- 描述： 机器人状态
- 消息格式
 
| 字段 | 类型 | 描述 |
| --- | --- | --- |
| mainPhase | uint8 | 机器人主状态 |
| subPhase | uint8 | 机器人子状态 |


NOTE:

mainPhase:

```c++
std::map<std::string, mainPhase_t> phase_map = {
    {"P_stand", P_stand},
    {"P_walk", P_walk},
    {"P_jump", P_jump},
    {"P_squat", P_squat},
    {"P_ERROR", P_ERROR},
    {"P_None", P_None}};
```

subPhase:

```c++
std::map<std::string, subPhase_t> sub_phase_map = {

    // walk
    {"walk_pre", walk_pre},
    {"walking", walking},
    {"walk_stop", walk_stop},

    // jump
    {"jump_pre", jump_pre},
    {"jump_take_off", jump_take_off},
    {"jump_flight", jump_flight},
    {"jump_touch_down", jump_touch_down},
    {"jump_to_ready", jump_to_ready},

    {"sub_phase_none", sub_phase_none},

};
```


<br></br>

### robot_imu_gyro_acc

- 话题名称：/robot_imu_gyro_acc
- 话题类型：dynamic_biped/robotImuGyroAcc
- 描述： 机器人 IMU 数据
- 消息格式

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| gyro | geometry_msgs/Vector3 | IMU 角速度 |
| acc | geometry_msgs/Vector3 | IMU 加速度 |

<br></br>


### robot_torso_state

- 话题名称：/robot_torso_state
- 话题类型：dynamic_biped/robotTorsoState
- 描述： 机器人躯干状态
- 消息格式

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| torsoR | geometry_msgs/Vector3 | 躯干旋转角度 |
| torsoRd | geometry_msgs/Vector3 | 躯干旋转速度 |
| torsoRdd | geometry_msgs/Vector3 | 躯干旋转加速度 |
| r | geometry_msgs/Vector3 | 质心旋转角度 |
| rd | geometry_msgs/Vector3 | 质心旋转速度 |
| rdd | geometry_msgs/Vector3 | 质心旋转加速度 |

NOTE:

- 所有角度单位为弧度
  
<br></br>


### robot_q_v_tau

- 话题名称：/robot_q_v_tau
- 话题类型：dynamic_biped/robotQVTau
- 描述： 机器人躯干 POSE 和 下肢关节数据
- 消息格式

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| q | float64[] | 位置 |
| v | float64[] | 速度 |
| tau | float64[] | 力矩 |

NOTE:

- `q` 的前 7 位躯干的位姿，（0-3）为四元数，（4-6）为位置，关节角度(7-19)，关节单位为 rad
- `v` 的前 6 位为躯干的角速度(0-2)、躯干速度(3-5), 关节角速度(6-18) 单位为 rad/s
- `tau` 为空
- 下肢的关节从左到右，从髋部到脚踝，共 12 个关节 `[l_leg_roll, l_leg_yaw, l_leg_pitch, l_knee, l_foot_pitch, l_foot_roll, r_leg_roll, r_leg_yaw, r_leg_pitch, r_knee, r_foot_pitch, r_foot_roll]`, 示意图见下方


<br></br>

### robot_arm_q_v_tau

- 话题名称：/robot_arm_q_v_tau
- 话题类型：dynamic_biped/robotArmQVVD
- 描述： 机器人手臂关节数据
- 消息格式

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| q | float64[] | 位置 |
| v | float64[] | 速度 |
| vd | float64[] | 加速度 |

NOTE:

- `q` 为手臂关节角度，单位为 rad
- `v` 为手臂关节角速度，单位为 rad/s
- `vd` 为手臂关节角加速度，单位为 rad/s^2
- 关节顺序从左到右，从肩部到手腕，共 14 个关节 `[l_arm_pitch, l_arm_roll, l_arm_yaw, l_forearm_pitch, l_forearm_yaw, l_hand_roll, l_hand_pitch, r_arm_pitch, r_arm_roll, r_arm_yaw, r_forearm_pitch, r_forearm_yaw, r_hand_roll, r_hand_pitch]`, 示意图见下方


#### KUAVO 机器人关节说明

- 单手臂拥有 7 个自由度，包括肩部 pitch、肩部 yaw、肩部 roll、小臂 pitch、小臂 yaw、手腕 pitch、手腕 roll。
- 单腿拥有 6 个自由度，包括髋部 pitch、髋部 yaw、髋部 roll、膝关节、脚踝 pitch、脚踝 roll。

上半身手臂关节名字：

```python
left_arm_joint_names = [
    "l_arm_pitch",
    "l_arm_roll",
    "l_arm_yaw",
    "l_forearm_pitch",
    "l_forearm_yaw",
    "l_hand_roll",
    "l_hand_pitch",
]

right_arm_joint_names = [
    "r_arm_pitch",
    "r_arm_roll",
    "r_arm_yaw",
    "r_forearm_pitch",
    "r_forearm_yaw",
    "r_hand_roll",
    "r_hand_pitch",
]
```

下半身腿部关节名字：

```python
left_leg_joint_names = [
    "l_leg_roll",
    "l_leg_yaw",
    "l_leg_pitch",
    "l_knee",
    "l_foot_pitch",
    "l_foot_roll",
]

right_leg_joint_names = [
    "r_leg_roll",
    "r_leg_yaw",
    "r_leg_pitch",
    "r_knee",
    "r_foot_pitch",
    "r_foot_roll",
]
```
