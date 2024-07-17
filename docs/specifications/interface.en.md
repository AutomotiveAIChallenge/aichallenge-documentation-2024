# Interface

## List

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
| Publisher    | `/aichallenge/objects`               | `sstd_msgs/msg/Float64MultiArray`                        |

<!--
| Subscription | `/vehicle/status/actuation_status`   | `tier4_vehicle_msgs/msg/ActuationCommandStamped`          |
| Publisher | `/vehicle/status/actuation_status`   | `tier4_vehicle_msgs/msg/ActuationStatusStamped`          |
-->

### `/control/command/control_cmd`

| Name                                | Description           |
| ----------------------------------- | --------------------- |
| stamp                               | Message timestamp     |
| lateral.stamp                       | Unused                |
| lateral.steering_tire_angle         | Target steering angle |
| lateral.steering_tire_rotation_rate | Unused                |
| longitudinal.stamp                  | Unused                |
| longitudinal.speed                  | Unused                |
| longitudinal.acceleration           | Target acceleration   |
| longitudinal.jerk                   | Unused                |

### `/vehicle/status/velocity_status`

| Name                  | Description              |
| --------------------- | ------------------------ |
| header.stamp          | Data acquisition time    |
| header.frame_id       | Frame ID (`base_link`)   |
| longitudinal_velocity | Longitudinal velocity    |
| lateral_velocity      | Lateral velocity         |
| heading_rate          | Angular velocity         |

### `/vehicle/status/steering_status`

| Name                | Description           |
| ------------------- | --------------------- |
| stamp               | Data acquisition time |
| steering_tire_angle | Steering angle        |

### `/control/command/gear_cmd`

| Name    | Description          |
| ------- | -------------------- |
| stamp   | Message timestamp    |
| command | Gear type            |

### `/vehicle/status/gear_status`

| Name   | Description           |
| ------ | --------------------- |
| stamp  | Data acquisition time |
| report | Gear type             |

### `/sensing/gnss/pose_with_covariance`

| Name                  | Description                         |
| --------------------- | ----------------------------------- |
| header.stamp          | Data acquisition time               |
| header.frame_id       | Frame ID (`map`)                    |
| pose.pose.position    | Vehicle position (origin of `base_link`) |
| pose.pose.orientation | Unused                              |
| pose.covariance       | Position accuracy                   |

### `/sensing/imu/imu_raw`

| Name                | Description             |
| ------------------- | ----------------------- |
| header.stamp        | Data acquisition time   |
| header.frame_id     | Frame ID (`imu_link`)   |
| orientation         | Orientation             |
| angular_velocity    | Angular velocity        |
| linear_acceleration | Linear acceleration     |

### `/aichallenge/objects`

| Name            | Description               |
| --------------- | ------------------------- |
| data[N * 4 + 0] | X coordinate of Nth object |
| data[N * 4 + 1] | Y coordinate of Nth object |
| data[N * 4 + 2] | Z coordinate of Nth object |
| data[N * 4 + 3] | Radius of Nth object      |
