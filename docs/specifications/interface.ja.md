# インターフェース

## 一覧

| Interface    | Name                                 | Type                                                     |
| ------------ | ------------------------------------ | -------------------------------------------------------- |
| Service      | `/control/control_mode_request`      | `autoware_auto_vehicle_msgs/srv/ControlModeCommand`      |
| Publisher    | `/vehicle/status/control_mode`       | `autoware_auto_vehicle_msgs/msg/ControlModeReport`       |
| Subscription | `/control/command/control_cmd`       | `autoware_auto_control_msgs/msg/AckermannControlCommand` |
| Publisher    | `/vehicle/status/velocity_status`    | `autoware_auto_vehicle_msgs/msg/VelocityReport`          |
| Publisher    | `/vehicle/status/steering_status`    | `autoware_auto_vehicle_msgs/msg/SteeringReport`          |
| Subscription | `/control/command/gear_cmd`          | `autoware_auto_vehicle_msgs/msg/GearCommand`             |
| Publisher    | `/vehicle/status/gear_status`        | `autoware_auto_vehicle_msgs/msg/GearReport`              |
| Publisher    | `/sensing/gnss/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped`            |
| Publisher    | `/sensing/imu/imu_raw`               | `sensor_msgs/msg/Imu`                                    |
| Publisher    | `/aichallenge/objects`               | `std_msgs.msg.Float64MultiArray`                         |
| Publisher    | `/aichallenge/pitstop/area`          | `std_msgs.msg.Float64MultiArray`                         |
| Publisher    | `/aichallenge/pitstop/condition`     | `std_msgs.msg.Int32`                                     |
| Publisher    | `/aichallenge/pitstop/status`        | `std_msgs.msg.Float32`                                   |

<!--
| Subscription | `/vehicle/status/actuation_status`   | `tier4_vehicle_msgs/msg/ActuationCommandStamped`          |
| Publisher | `/vehicle/status/actuation_status`   | `tier4_vehicle_msgs/msg/ActuationStatusStamped`          |
-->

### `/control/command/control_cmd`

| Name                                | Description          |
| ----------------------------------- | -------------------- |
| stamp                               | メッセージの送信時刻 |
| lateral.stamp                       | 未使用               |
| lateral.steering_tire_angle         | 目標操舵角           |
| lateral.steering_tire_rotation_rate | 未使用               |
| longitudinal.stamp                  | 未使用               |
| longitudinal.speed                  | 未使用               |
| longitudinal.acceleration           | 目標加速度           |
| longitudinal.jerk                   | 未使用               |

### `/vehicle/status/velocity_status`

| Name                  | Description              |
| --------------------- | ------------------------ |
| header.stamp          | データの取得時刻         |
| header.frame_id       | フレームID (`base_link`) |
| longitudinal_velocity | 縦速度                   |
| lateral_velocity      | 横速度                   |
| heading_rate          | 角速度                   |

### `/vehicle/status/steering_status`

| Name                | Description      |
| ------------------- | ---------------- |
| stamp               | データの取得時刻 |
| steering_tire_angle | 操舵角           |

### `/control/command/gear_cmd`

| Name    | Description          |
| ------- | -------------------- |
| stamp   | メッセージの送信時刻 |
| command | ギアの種類           |

### `/vehicle/status/gear_status`

| Name   | Description      |
| ------ | ---------------- |
| stamp  | データの取得時刻 |
| report | ギアの種類       |

### `/sensing/gnss/pose_with_covariance`

| Name                  | Description                       |
| --------------------- | --------------------------------- |
| header.stamp          | データの取得時刻                  |
| header.frame_id       | フレームID (`map`)                |
| pose.pose.position    | 車両位置 (`base_link` 原点の位置) |
| pose.pose.orientation | 未使用                            |
| pose.covariance       | 位置精度                          |

### `/sensing/imu/imu_raw`

| Name                | Description             |
| ------------------- | ----------------------- |
| header.stamp        | データの取得時刻        |
| header.frame_id     | フレームID (`imu_link`) |
| orientation         | 方位                    |
| angular_velocity    | 角速度                  |
| linear_acceleration | 加速度                  |

### `/aichallenge/objects`

| Name            | Description              |
| --------------- | ------------------------ |
| data[N * 4 + 0] | N番目の仮想障害物のX座標 |
| data[N * 4 + 1] | N番目の仮想障害物のY座標 |
| data[N * 4 + 2] | N番目の仮想障害物のZ座標 |
| data[N * 4 + 3] | N番目の仮想障害物の半径  |

### `/aichallenge/pitstop/area`

| Name    | Description                                   |
| ------- | --------------------------------------------- |
| data[0] | ピットストップエリア中心のX座標               |
| data[1] | ピットストップエリア中心のY座標               |
| data[2] | ピットストップエリア中心のZ座標               |
| data[3] | ピットストップエリアの方向のクオータニオンX値 |
| data[4] | ピットストップエリアの方向のクオータニオンY値 |
| data[5] | ピットストップエリアの方向のクオータニオンZ値 |
| data[6] | ピットストップエリアの方向のクオータニオンW値 |
| data[7] | ピットストップエリアのX方向のサイズ           |
| data[8] | ピットストップエリアのY方向のサイズ           |

### `/aichallenge/pitstop/condition`

| Name | Description            |
| ---- | ---------------------- |
| data | 車両のコンディション値 |

### `/aichallenge/pitstop/status`

| Name | Description                            |
| ---- | -------------------------------------- |
| data | ピットストップの判定が成立している秒数 |
