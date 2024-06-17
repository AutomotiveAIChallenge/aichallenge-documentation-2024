# シミュレーター

## 概要

このページではAIチャレンジで使用されるシミュレーターの仕様について説明します。

シミュレーターは、Autowareのためのオープンソース自動運転シミュレーター「[AWSIM](https://github.com/tier4/AWSIM)」をベースとして作成されています。

## 車両（レーシングカート）

車両はAWSIMにおける[EGO Vehicle](https://tier4.github.io/AWSIM/Components/Vehicle/EgoVehicle/)の仕様に準拠しており、実際のレーシングカートに近いスペックで作成されています。

![vehicle-appearance](./images/vehicle-appearance.png)

### パラメータ

車両のパラメータを以下の表にまとめています。

| **項目**               | **値**    |
| ---------------------- | --------- |
| 車両重量               | 160 kg    |
| 全長                   | 200 cm    |
| 全幅                   | 145 cm    |
| 前輪タイヤ直径         | 24 cm     |
| 前輪タイヤ幅           | 13 cm     |
| 前輪ホイールトレッド   | 93 cm     |
| 後輪タイヤ直径         | 24 cm     |
| 後輪タイヤ幅           | 18 cm     |
| 後輪ホイールトレッド   | 112 cm    |
| 最大ステアリング転舵角 | 80 °      |
| 駆動時最大加速度       | 3.2 m/s^2 |

#### Vehicleコンポーネント

Vehicleコンポーネントの設定内容を以下の表にまとめています。各項目の詳細については[こちらのマニュアル](https://tier4.github.io/AWSIM/Components/Vehicle/EgoVehicle/#vehicle-script)をご覧ください。

| **項目**                            | **値** |
| ----------------------------------- | ------ |
| Use Inertia                         | Off    |
| **Physics Settings (experimental)** |        |
| Sleep Velocity Threshold            | 0.02   |
| Sleep Time Threshold                | 0      |
| Skidding Cancel Rate                | 0.236  |
| **Input Settings**                  |        |
| Max Steer Angle Input               | 80     |
| Max Acceleration Input              | 3.2    |

#### Rigidbodyコンポーネント

Rigidbodyコンポーネントの設定内容を以下の表にまとめています。詳細については[こちらのマニュアル](https://tier4.github.io/AWSIM/Components/Vehicle/EgoVehicle/#rigidbody)をご覧ください。

| **項目**     | **値** |
| ------------ | ------ |
| Mass         | 160    |
| Drag         | 0      |
| Angular Drag | 0      |

### CoM位置

CoM(Center of Mass)は、車両Rigidbodyの質量中心です。CoM位置は、車両の中心かつ車輪軸の高さに設定されています。

![side-view-of-com](./images/side-view-of-com.png)

![top-view-of-com](./images/top-view-of-com.png)

### 車両コライダー

車両コライダーは、車両と他オブジェクトやチェックポイントとの接触判定に利用されます。車両コライダーは車両オブジェクトのメッシュをベースとして作成されています。

![body-collider](./images/body-collider.png)

### ホイールコライダー

車両には各車輪に1つずつ、合計4つのホイールコライダーが設定されており、等価二輪モデルではなく四輪モデルでの車両シミュレーションが行われています。

![wheel-collider](./images/wheel-collider.png)

ホイールコライダーは以下のように設定されています。ホイールコライダーの詳細については[こちらのマニュアル](https://tier4.github.io/AWSIM/Components/Vehicle/EgoVehicle/#wheels-colliders)をご覧ください。

| **項目**              | **値** |
| --------------------- | ------ |
| Mass                  | 1      |
| Radius                | 0.12   |
| Wheel Damping Rate    | 0.25   |
| Suspension Distance   | 0.001  |
| **Suspension Spring** |        |
| Spring (N/m)          | 35000  |
| Damper (N\*s/m)       | 3500   |
| Target Position       | 0.01   |

### センサ構成

TODO
