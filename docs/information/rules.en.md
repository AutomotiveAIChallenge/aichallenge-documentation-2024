# Rules

## Overview

Teams will compete to achieve the shortest driving time while completing the specified number of laps on a designated course.

## Environment

The course will have a "Start Area," "Control Line," and "Pit Stop Area." Vehicles will start from the Start Area, and the driving time will be measured when they touch the Control Line. For details on the Pit Stop Area, refer to the "Pit Stop" section below. Each team will drive individually, without other vehicles or obstacles on the course simultaneously.

## Progress

Each team will have a preparation session to set up their vehicle and a recording session to measure driving times. However, in the preliminary competition, vehicles will not be used, so there will be no preparation session. Advanced class teams can always perform vehicle maintenance, so they do not have a preparation session either.

| Item              | Final Competition | Preliminary Competition |
| ----------------- | ----------------- | ----------------------- |
| Preparation Session | TBD               | None                    |
| Recording Session  | TBD               | 7:00                    |
| Number of Laps     | TBD               | 6                       |

### Starting the Drive

Vehicles will start from the Start Area, and the driving time will begin when they first touch the Control Line. In the preliminary competition, vehicles will be pre-positioned in a predetermined posture. In the final competition, vehicles can be placed in any posture within the Start Area, but operations on the vehicle are only allowed within the Start Area.

### Ending the Drive

The drive will end and be recorded as a result under the following conditions:

- The specified number of laps is completed.
- The allotted time for the recording session has elapsed.
- The vehicle is touched and operated.
- Any other reason deemed appropriate by the organizers.

### Stopping the Drive

The drive will end and be invalidated under the following conditions:

- (Preliminary only) The vehicle has not passed the Control Line within 2 minutes from the start of the recording session.
- (Preliminary only) The vehicle has significantly deviated from the course.
- The course walls are moved.
- Any other reason deemed appropriate by the organizers.

<!--
### Retrying

In the final competition, if the vehicle cannot continue driving for some reason and you want to retry, you can apply to the staff for a retry. Applying for a retry will be treated as the end of the drive at that time, and the best lap count and driving time from all drives during the recording session will be adopted.
-->

## Ranking

The ranking will be determined based on the following criteria:

- If the specified number of laps is completed, the team with the shortest driving time.
- If the specified number of laps is not completed:
  - The team with the most laps.
  - If the lap count is the same, the team with the shortest time to the last lap.

## Pit Stop

Vehicles have a virtual value called "Condition," which, when increased, restricts their speed. Condition increases as the vehicle drives and also when it collides with virtual obstacles described below. The Condition can be reset to its initial value by stopping in the Pit Stop Area for a specified number of seconds.

| Setting Item            | Value        | Additional Notes               |
| ----------------------- | ------------ | ------------------------------ |
| Pit Stop Time           | 3.0 seconds  | ―                              |
| Speed Limit Activation  | 1000         | Maximum speed is limited to 20 km/h |
| Section Pass            | 30           | ―                              |
| Virtual Obstacle Collision | 20 - 380   | Varies depending on the collision |

### Pit Stop Area

The Pit Stop Area is indicated by a green frame as shown in the image below.

![pit-stop-area](./images/pit-stop-area.png)

### Increasing Condition

The course is virtually divided into multiple sections, and Condition increases by a fixed amount each time the vehicle exits a section. Additionally, virtual obstacles displayed with a purple frame, as shown in the image below, are placed on the course, and Condition increases if the vehicle collides with them (virtual obstacles do not affect the physical behavior of the vehicle).

Virtual obstacles are generated at random positions within a section each time the vehicle exits a section. After the first lap, virtual obstacles are removed and regenerated in the section, so multiple virtual obstacles will not be placed within the same section. Also, no virtual obstacles are generated near the Pit Stop Area.

![virtual-objects](./images/virtual-objects.png)
