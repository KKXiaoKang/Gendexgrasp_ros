## KUAVO机器人节点提供的服务

### setPhase

- 服务名称：/setPhase
- 服务类型：dynamic_biped/srvChangePhases
- 描述：设置机器人状态机
  
- 请求

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| masterID | uint8 | 主状态机 ID |
| stateReq | string | 请求的状态 |
| subState | string | 子状态 |

- 响应

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| stateRes | int16 | 结果 |

Note:

- masterID: 主状态机 ID (无需填写)
- stateReq: 请求的状态(如 `P_stand`、`P_walk`、`P_jump`、`P_squat`)
- subState: 子状态(如果 stateReq 为 `P_jump`，则 subState 为 `jump_pre` 或 `jump_take_off`)

<br></br>

### change_ctl_mode

- 服务名称：/change_ctl_mode
- 服务类型：dynamic_biped/srvchangeCtlMode
- 描述：切换控制模式(位置控制/速度控制)

- 请求

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| control_mode | uint8 | 控制模式 |

- 响应

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| control_mode | uint8 | 控制模式 |

Note:

- control_mode: 0->PositionControl | 1->VelocityControl

<br></br>


### change_AMBAC_ctrl_mode

- 服务名称：/change_AMBAC_ctrl_mode
- 服务类型：dynamic_biped/changeAMBACCtrlMode
- 描述：机器人启动默认会蹲起，调用此服务可以让机器人站立

- 请求

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| control_mode | bool | 控制模式 |

- 响应

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| result | bool | 结果 |

Note:

- control_mode: true->Stand

<br></br>


#### control_jodell_claw_position

- 服务名称：/control_jodell_claw_position
- 服务类型：dynamic_biped/controlJodellClaw
- 描述：控制均舵夹爪的开合位置

- 请求

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| left_claw_position | uint8 | 左夹爪位置 |
| right_claw_position | uint8 | 右夹爪位置 |

- 响应

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| result | bool | 结果 |

Note:

- left_claw_position: 左夹爪位置，范围在 0-255，0 为张开，255 为闭合(无单位)
- right_claw_position: 右夹爪位置，范围在 0-255，0 为张开，255 为闭合(无单位)

### update_center_of_mass_position

- 服务名称：/update_center_of_mass_position
- 服务类型：dynamic_biped/comPosition
- 描述：更新机器人质心位置

- 请求

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| position | geometry_msgs/Vector3 | 位置 |
| time | float32 | 时间 |
| acc | float32 | 加速度 |

- 响应

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| success | bool | 结果 |

Note:

- **position**：包含质心位置的`geometry_msgs/Vector3`消息， 单位为米。其中，`x`分量代表质心在 X 轴上的位置，`y`分量代表质心在 Y 轴上的位置，`z`分量代表质心在 Z 轴上的位置。(例如将 `z` 设得比原来低，机器人就会向下蹲, `x` 设得比原来前，机器人就会向前倾)
- **time**：执行的时间，单位为秒。默认值为 0，不起作用，由加速度控制插值，如果设定时间，则使用用户的时间来进行插值，忽略加速度。
- **acc**：执行的加速度，单位为米/秒²。默认值为 0.5。


<br></br>

### get_center_of_mass_position

- 服务名称：/get_center_of_mass_position
- 服务类型：dynamic_biped/getComPosition
- 描述：获取机器人质心位置

- 请求 (无需填写)

- 响应

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| com_value | geometry_msgs/Vector3 | 质心位置 |

Note:

- **com_value**：包含质心位置的`geometry_msgs/Vector3`消息。其中，`x`分量代表质心在 X 轴上的位置，`y`分量代表质心在 Y 轴上的位置，`z`分量代表质心在 Z 轴上的位置。(单位：米)

<br></br>

### change_hand_arm_poses_by_config_name

- 服务名称：/change_hand_arm_poses_by_config_name
- 服务类型：dynamic_biped/changeHandArmPosesByConfigName
- 描述：根据传入 POSE_CSV 文件名控制手臂和手部位置

- 请求

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| config_name | string | POSE_CSV 文件名 |

- 响应

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| result | bool | 结果 |

Note:

- config_name: POSE_CSV 文件名,需要配置文件目录(指的是 `$HOME/.config/lejuconfig/config/kuavo_v<VERSION>` 目录)下存在对应的 POSE_CSV 文件

<br></br>

### control_end_hand

- 服务名称：/control_end_hand
- 服务类型：dynamic_biped/controlEndHand
- 描述：控制机器人灵巧手位置

- 请求

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| left_hand_position | uint8[] | 左手位置 |
| right_hand_position | uint8[] | 右手位置 |

- 响应

| 字段 | 类型 | 描述 |
| --- | --- | --- |
| result | bool | 结果 |


Note:

- hand_posistion: 数组长度为 6，对应大拇指关节，拇指外展肌，食指关节, 中指关节，无名指关节，小指关节, 位置范围：0-100, 0 为张开，100 为闭合
