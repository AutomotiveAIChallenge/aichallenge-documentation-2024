# シミュレーター

## 概要
このページではAIチャレンジで使用されるシミュレーターの仕様について説明します。

シミュレーターは、Autowareのためのオープンソース自動運転シミュレーター「[AWSIM](https://github.com/tier4/AWSIM)」をベースとして作成されています。

## 車両（ゴーカート）
車両はAWSIMにおける[EGO Vehicle](https://tier4.github.io/AWSIM/Components/Vehicle/EgoVehicle/)の仕様に準拠しており、実際のゴーカートに近いスペックで作成されています。

![vehicle-appearance](./images/vehicle-appearance.png)

### パラメータ
車両のパラメータを以下の表にまとめています。

| **項目**                        | **値**                     |
| ------------------ | --------- |
| 車両重量             | 160 kg    |
| 全長                | 200 cm    |
| 全幅                | 145 cm    |
| 前輪タイヤ直径        | 24 cm     |
| 前輪タイヤ幅          | 13 cm     |
| 前輪ホイールトレッド   | 93 cm     |
| 後輪タイヤ直径        | 24 cm     |
| 後輪タイヤ幅          | 18 cm    |
| 後輪ホイールトレッド   | 112 cm    |
| 最大ステアリング転舵角 | 80 °      |
| 駆動時最大加速度      | 3.2 m/s^2 |

#### Vehicle スクリプト
Vehicle スクリプトの内容を以下の表にまとめています。詳細については[こちらのマニュアル](https://tier4.github.io/AWSIM/Components/Vehicle/EgoVehicle/#vehicle-script)をご覧ください。

| **項目**                        | **値**                     |
| ------------------------------- | ------------------------- |
| **Vehicle Settings**            |                           |
| Center Of Mass Transform        | CoM (Transform)           |
| Use Inertia                     | Off                         |
| Inertia                         | X: 15, Y: 25, Z: 20       |
| **Physics Settings (experimental)** |                      |
| Sleep Velocity Threshold        | 0.02                      |
| Sleep Time Threshold            | 0                         |
| Skidding Cancel Rate            | 0.236                     |
| **Axles Settings**              |                           |
| **Front Axle**                  |                           |
| Left Wheel                      | FrontLeftWheel (Wheel)    |
| Right Wheel                     | FrontRightWheel (Wheel)   |
| **Rear Axle**                   |                           |
| Left Wheel                      | RearLeftWheel (Wheel)     |
| Right Wheel                     | RearRightWheel (Wheel)    |
| **Input Settings**              |                           |
| Max Steer Angle Input           | 80                        |
| Max Acceleration Input          | 3.2                       |
| **Inputs**                      |                           |
| Automatic Shift Input           | DRIVE                     |
| Acceleration Input              | 0                         |
| Steer Angle Input               | 0                         |
| Signal Input                    | NONE                      |

#### Rigidbody コンポーネント
Rigidbody コンポーネントの内容を以下の表にまとめています。詳細については[こちらのマニュアル](https://tier4.github.io/AWSIM/Components/Vehicle/EgoVehicle/#rigidbody)をご覧ください。

| **項目**                | **値**          |
|-------------------------|-----------------|
| Mass                | 160             |
| Drag                | 0               |
| Angular Drag        | 0               |
| Use Gravity         | On              |
| Is Kinematic        | Off             |
| Interpolate         | Interpolate     |
| Collision Detection | Continuous Dynamic |
| **Constraints**         |                 |
| Freeze Position     | X: Off, Y: Off, Z: Off |
| Freeze Rotation     | X: Off, Y: Off, Z: Off |
| **Info**                |                 |
| Speed                   | 0               |
| Velocity                | X: 0, Y: 0, Z: 0 |
| Angular Velocity        | X: 0, Y: 0, Z: 0 |
| Inertia Tensor      | X: 38.50399, Y: 57.13089, Z: 23.12513 |
| Inertia Tensor Rotation | X: 0.069100, Y: 359.208, Z: 359.8651 |
| Local Center of Mass| X: 0.002359, Y: 0.2155101, Z: 0.0887787 |
| World Center of Mass| X: 375.2133, Y: 7.30751, Z: -4.844843 |
| Sleep State         | Awake           |

### CoM位置
CoM(Center of Mass)は、車両Rigidbodyの質量中心です。CoM位置は、車両の中心かつ車輪軸の高さに設定されています。

![side-view-of-com](./images/side-view-of-com.png)

![top-view-of-com](./images/top-view-of-com.png)

### コライダー
Collidersオブジェクトの構成は以下の通りです。

![colliders-object](./images/colliders-object.png)

コライダーは、車両が他のオブジェクトと接触できるように設定します。MeshColliderコンポーネントは車両オブジェクトのメッシュを取得し、それに基づいてコライダーを構築します。

![body-collider](./images/body-collider.png)

#### Mesh Collider コンポーネント
Mesh Collider コンポーネントの内容を以下の表にまとめています。詳細については[こちらのマニュアル](https://tier4.github.io/AWSIM/Components/Vehicle/EgoVehicle/#bodycollider)をご覧ください。

| **項目**               | **値**                   |
| ----------------- | ------------------- |
| Convex            | On         |
| Is Trigger        | Off    |
| Cooking Options   | Cook For Faster Simulation |
|   　　　　　　　　 | Enable Mesh Cleaning |
|   　　　　　　　　 | Weld Colocated Vertices |
|   　　　　　　　　 | Use Fast Midphase |
| Material          | None (Physic Material) |
| Mesh              | Collision           |

### ホイールコライダー
車両は、車輪に関連する視覚オブジェクトとともに、各車輪に1つずつ、合計4つのホイールコライダーを持ちます。ホイールコライダーが、道路と接触する唯一の部分です。なお、シミュレーションは等価二輪モデルではなく、等価四輪モデルで行っています。

![wheel-collider](./images/wheel-collider.png)

Wheelスクリプトは特定の車輪のコライダーと視覚オブジェクトへの参照を提供します。これにより、Vehicleスクリプトは各車輪に対して以下のようなアクションを実行できるようになります：

- ホイールコライダーのステアリング角を更新する
- 速度とターンの角度に応じて車輪の視覚部分を更新する
- WheelHitオブジェクトに保存された車輪の接触情報を更新する
- 加速（スキッドのキャンセルを含む）に応じてタイヤが前方および側方に及ぼす力を更新する
- タイヤのスリープ設定を保証する（Rigidbodyをスリープ状態にすることは不可能ですが、すべての車輪をスリープ状態にすることで、この効果に近づけます）。

Wheel Collider Configスクリプトは、ホイールコライダーに対するインスペクター入力を防ぎ、摩擦を0に設定し、車輪のサスペンションと衝突のみを有効にするために設定されています。ホイールコライダーの詳細については[こちらのマニュアル](https://tier4.github.io/AWSIM/Components/Vehicle/EgoVehicle/#wheels-colliders)をご覧ください。

#### Wheel Collider コンポーネント
| **項目**                        | **値**                              |
| -------------------------- | ------------------------------ |
| Mass                       | 1                              |
| Radius                     | 0.12                           |
| Wheel Damping Rate         | 0.25                           |
| Suspension Distance        | 0.001                          |
| Force App Point Distance   | 0                             |
| Center (X, Y, Z)           | (0, 0, 0)                      |
| **Suspension Spring**      |                                |
| Spring (N/m)               | 35000                          |
| Damper (N*s/m)             | 3500                           |
| Target Position            | 0.01                           |
| **Forward Friction**       |                                |
| Extremum Slip              | 0                              |
| Extremum Value             | 0                              |
| Asymptote Slip             | 0                              |
| Asymptote Value            | 0                              |
| Stiffness                  | 0                              |
| **Sideways Friction**      |                                |
| Extremum Slip              | 0                              |
| Extremum Value             | 0                              |
| Asymptote Slip             | 0                              |
| Asymptote Value            | 0                              |
| Stiffness                  | 0                              |

#### Wheel スクリプト

| **項目**                      | **値**                                    |
| ------------------------ | -------------------------------------- |
| Wheel Collider           | RearLeftWheel (Wheel Collider)         |
| Wheel Visual Transform   | RearLeftWheel (Transform)              |

#### Wheel Collider Config スクリプト

| **項目**                      | **値**         |
| ------------------------ | --------- |
| Radius (m)               | 0.12      |
| Suspension Distance (m)  | 0.001     |
| **Suspension Spring**    |           |
| Spring (N/m)             | 35000     |
| Damper (N*s/m)           | 3500      |
| Target Position          | 0.01      |


### センサ構成
TODO