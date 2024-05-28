# Simulator

## Overview
This page describes the specifications of the simulator used in the AI Challenge.

The simulator is based on the open-source autonomous driving simulator "[AWSIM](https://github.com/tier4/AWSIM)" developed for Autoware.

## Vehicle (Go-Kart)
The vehicle conforms to the specifications of the [EGO Vehicle](https://tier4.github.io/AWSIM/Components/Vehicle/EgoVehicle/) in AWSIM and is designed with specifications close to an actual go-kart.

![vehicle-appearance](./images/vehicle-appearance.png)

### Parameters

![vehicle-component](./images/vehicle-component.png)

![rigidbody-component](./images/rigidbody-component.png)

| Item                  | Value      |
| --------------------- | ---------- |
| Vehicle Weight        | 160 kg     |
| Length                | 200 cm     |
| Width                 | 145 cm     |
| Front Wheel Diameter  | 24 cm      |
| Front Wheel Width     | 13 cm      |
| Front Wheel Tread     | 93 cm      |
| Rear Wheel Diameter   | 24 cm      |
| Rear Wheel Width      | 18 cm      |
| Rear Wheel Tread      | 112 cm     |
| Maximum Steering Angle| 80°        |
| Maximum Acceleration  | 3.2 m/s^2  |

### CoM Position
CoM (Center of Mass) is the mass center of the vehicle Rigidbody. The CoM position is set at the center of the vehicle and at the height of the wheel axles.

![side-view-of-com](./images/side-view-of-com.png)

![top-view-of-com](./images/top-view-of-com.png)

### Colliders
The configuration of the Colliders object is as follows.

![colliders-object](./images/colliders-object.png)

Colliders ensure that the vehicle can make contact with other objects. The MeshCollider component takes the mesh of the vehicle object and constructs a collider based on it.

![mesh-collider-component](./images/mesh-collider-component.png)

![body-collider](./images/body-collider.png)

### Wheel Colliders
The vehicle has a total of four wheel colliders, one for each wheel, in addition to the visual objects related to the wheels. Wheel colliders are the only parts that make contact with the road.

The Wheel (script) provides a reference to the collider and visual object for the particular wheel. This allows the Vehicle (script) to perform certain actions on each of the wheels, such as:

- Update the steering angle in the WheelCollider.
- Update the visual part of the wheel depending on the speed and angle of the turn.
- Update the wheel contact information stored in the WheelHit object.
- Update the force exerted by the tire forward and sideways depending on the acceleration (including cancellation of skidding).
- Ensure setting the tire sleep (it is impossible to put Rigidbody to sleep, but putting all wheels to sleep allows to get closer to this effect).

The Wheel Collider Config script is designed to prevent inspector input for wheel colliders, set friction to zero, and only enable wheel suspension and collisions. For more details on wheel colliders, please refer to [this manual](https://docs.unity3d.com/Manual/class-WheelCollider.html).

![wheel-collider-component](./images/wheel-collider-component.png)

![wheel-collider](./images/wheel-collider.png)

### Sensor Configuration
TODO