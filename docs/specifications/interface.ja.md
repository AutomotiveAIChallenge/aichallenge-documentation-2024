# インターフェース

## 一覧

| Interface    | Name                                 | Type                                                     |
| ------------ | ------------------------------------ | -------------------------------------------------------- |
| Subscription | `/control/command/control_cmd`       | `autoware_auto_control_msgs/msg/AckermannControlCommand` |
| Publisher    | `/vehicle/status/velocity_status`    | `autoware_auto_vehicle_msgs/msg/VelocityReport`          |
| Publisher    | `/vehicle/status/steering_status`    | `autoware_auto_vehicle_msgs/msg/SteeringReport`          |
| Subscription | `/control/command/gear_cmd`          | `autoware_auto_vehicle_msgs/msg/GearCommand`             |
| Publisher    | `/vehicle/status/gear_status`        | `autoware_auto_vehicle_msgs/msg/GearReport`              |
| Publisher    | `/sensing/gnss/pose_with_covariance` | `geometry_msgs/msg/PoseWithCovarianceStamped`            |
| Publisher    | `/sensing/imu/imu_data`              | `sensor_msgs/msg/Imu`                                    |

<!--
| Service      | `/control/control_mode_request`      | `autoware_auto_vehicle_msgs/srv/ControlModeCommand`      |
| Publisher    | `/vehicle/status/control_mode`       | `autoware_auto_vehicle_msgs/msg/ControlModeReport`       |
| Publisher    | `/vehicle/status/actuation_status`   | `tier4_vehicle_msgs/msg/ActuationStatusStamped`          |
-->

### `/control/command/control_cmd`

| Name                                | Description          |
| ----------------------------------- | -------------------- |
| stamp                               | メッセージの送信時刻 |
| lateral.stamp                       | 未使用               |
| lateral.steering_tire_angle         | T.B.D.               |
| lateral.steering_tire_rotation_rate | T.B.D.               |
| longitudinal.stamp                  | 未使用               |
| longitudinal.speed                  | T.B.D.               |
| longitudinal.acceleration           | T.B.D.               |
| longitudinal.jerk                   | 未使用               |

### `/vehicle/status/velocity_status`

| Name                  | Description              |
| --------------------- | ------------------------ |
| header.stamp          | データの取得時刻         |
| header.frame_id       | フレームID (`base_link`) |
| longitudinal_velocity | 速度                     |
| lateral_velocity      | T.B.D.                   |
| heading_rate          | T.B.D                    |

### `/vehicle/status/steering_status`

| Name                | Description      |
| ------------------- | ---------------- |
| stamp               | データの取得時刻 |
| steering_tire_angle | タイヤ角度       |

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

| Name                  | Description                                  |
| --------------------- | -------------------------------------------- |
| header.stamp          | データの取得時刻                             |
| header.frame_id       | フレームID (`map`)                           |
| pose.pose.position    | 車両位置 (フレームID `base_link` 原点の位置) |
| pose.pose.orientation | 未使用                                       |
| pose.covariance       | 位置精度                                     |

### `/sensing/imu/imu_data`

| Name                | Description              |
| ------------------- | ------------------------ |
| header.stamp        | データの取得時刻         |
| header.frame_id     | フレームID (`base_link`) |
| orientation         | 方位                     |
| angular_velocity    | 角速度                   |
| linear_acceleration | 加速度                   |
