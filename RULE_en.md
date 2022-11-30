## Competition Rules

## Competition Details
This competition is to compete the driving time by the automatic vehicle on the online simulator.

## Driving Environment
![](https://user-images.githubusercontent.com/10482465/204548584-a893236a-913b-467b-ae8e-3bb712e3f122.png)
- The vehicle will drive on a portion of the nishi-Shinjuku area.

## Automated vehicle
![](https://user-images.githubusercontent.com/10482465/204553983-bc6ae7b2-c66d-4a11-80eb-123a5c3830c3.png)
- A Lexus RX450h will be used as the vehicle for autonomous driving.

## Rules for the entire competition
- You need to create source code to run through the scenario on [AWSIM](https://github.com/tier4/AWSIM)(simulator for automated driving) by utilizing [Autoware](https://github.com/autowarefoundation/autoware/tree/awsim-stable).
- Participants will create source code to be able to complete the given scenario and verify it in the local environment first.
- By uploading the source code to the online environment, the simulation will be run online. Finally, the ranking will be based on the time of the simulation results of the uploaded source code.

## Scenario
This section describes the events included in the scenario.

- The following information is common to both the online scoring environment and the local environment, but the order of events, vehicle locations, and other details differ.
- If a route or method other than those described for each event is used, it may not work in the online scoring environment.

### Events on the Challenge Course
#### Following a low speed vehicle
![](https://user-images.githubusercontent.com/10482465/204734102-696f8f1e-b0dd-48ce-b602-17b1992bfaa7.png)
- This is a situation where a low-speed vehicle is running in front of you.
- It is possible to run with the sample code, but the running time can be improved by improving the control to follow the slow vehicle according to its speed.

#### Avoiding a vehicle trying to park
![](https://user-images.githubusercontent.com/10482465/204734114-54d9bf42-759c-48a7-a107-71150d2d2cea.png)
- This is a situation to avoid a vehicle that is about to park.
- It is possible to run with the sample code, but the running time can be improved by improving the route plan so as to overtake the vehicle.

### Events on the Advanced Course
The following events have been added to the Advanced Course in addition to the events on the Challenge Course, as well as other NPC pedestrians and vehicles in the route.

#### Multiple Vehicle Avoidance
![](https://user-images.githubusercontent.com/10482465/204734079-d68e502f-7b9e-4890-bc12-614e19ad4708.png)
- This is a situation to avoid accidental and parked vehicles.
- It is not possible to run with the sample code. The driving route planning needs to be improved so that it can proceed between multiple vehicles.

#### Avoiding obstacles in the midst of vehicle traffic.
![](https://user-images.githubusercontent.com/10482465/204734117-7dd16c72-5521-4cdb-a4d3-54b8090d5a78.png)
- This is a situation in which a vehicle is passing from behind and an obstacle is being avoided.
- It is possible to run with the sample code, but the running time can be improved by improving the behavior so that the vehicle interrupts at the appropriate timing.

## Detailed Rules
### Goal Judgment
![](https://user-images.githubusercontent.com/10482465/204742349-6fa49680-f9fe-4589-9030-4bbd4a6d9cd3.png)
- Checkpoints (① to ③ in the above image) are located on the course. A vehicle is judged to have reached the goal when it passes all checkpoints in order from ① to ③.
- As shown in the image above, checkpoints are defined in the shape of a cube.
- When a checkpoint and a vehicle overlap, even slightly, it is judged to have passed the checkpoint.
- The goal judgment is made when the vehicle comes to a complete stop.
- Stopping at the finish line must be done before the stop line.
  - To be precise, the vehicle must be stopped 1 meter before the stop line with the leading edge of the vehicle between the stop line and the stop line.

### Time Measurement
- The timing when your vehicle starts moving is the timing to start measurement.
- Time measurement ends when the vehicle is judged to have reached the goal.

### Disqualification
The following conditions will result in disqualification.

- Collision with a vehicle, pedestrian, guardrail, etc.
- Crossing the stop line at the finish line
- Vehicle speed exceeds 50 km/h

### Checking the Time
You can check your time with the following command.
````
# In the Rocker container
source /aichallenge/aichallenge_ws/install/setup.bash
ros2 topic echo /score/result
# The following will be displayed when a goal is scored or disqualified.
# score: 164699
# has_finished: true
# has_collided: false
# has_park_failed: false
# check_point_count: 2
# the value of `score` is the final time.
# if `has_finished` is false, you are disqualified.
```
