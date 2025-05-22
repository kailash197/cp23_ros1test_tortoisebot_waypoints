# ROS Integration Test

## Build Instructions

### 1. Clone the repository
[Terminal 1]
```bash
cd ~/simulation_ws/src
git clone https://github.com/kailash197/cp23_ros1test_tortoisebot_waypoints.git
git checkout master

```
### 2. Use catkin make to build the project
[Terminal 1]
```bash
source /opt/ros/noetic/setup.bash
cd ~/simulation_ws && catkin_make --pkg tortoisebot_waypoints && source devel/setup.bash

```

## Start simulation

### 1. Launch the ROS1 simulation for the TortoiseBot
[Terminal 1]
```bash
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash
roslaunch tortoisebot_gazebo tortoisebot_playground.launch

```

**Note**: Abort and re-launch Gazebo if there are any issues. Use `kill -9 <gazebo_pid>`

## Test Instructions

### 1. Using launch files 

#### 1. Passing Conditions
Execute the test file and verify if all the tests are passing properly with passing conditions
The `waypoints_test.test` launch file must have `test_pass` parameter set to `True`.
```xml
  <!-- Test parameter -->
  <param name="test_pass" value="true"/>
```

[Terminal 3]
```bash
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash
rostest tortoisebot_waypoints waypoints_test.test --reuse-master

```

Expected output:
```bash
[ROSTEST]-----------------------------------------------------------------------

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

```

#### 2. Failing Conditions
Make the necessary changes in the program to test for failing conditions and execute the test file and verify if all the tests are failing properly with failing conditions.

Expected output:
```bash
[ROSTEST]-----------------------------------------------------------------------

SUMMARY
 * RESULT: FAIL
 * TESTS: 1
 * ERRORS: 1
 * FAILURES: 0

```

### 2. Launch the Waypoints Action Server for ROS1
[Terminal 2]
```bash
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash
rosrun tortoisebot_waypoints tortoisebot_action_server.py

```    





