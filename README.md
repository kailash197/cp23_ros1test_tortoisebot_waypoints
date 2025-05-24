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

## Test Instructions

### 1. Launch the ROS1 simulation for the TortoiseBot
[Terminal 1]
```bash
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash
roslaunch tortoisebot_gazebo tortoisebot_playground.launch

```

**Note**: Abort and re-launch Gazebo if there are any issues. Use `kill -9 <gazebo_pid>`

### 2. Launch the Waypoints Action Server for ROS1
[Terminal 2]
```bash
source /opt/ros/noetic/setup.bash
cd ~/simulation_ws && catkin_make && source devel/setup.bash
rosrun tortoisebot_waypoints tortoisebot_action_server.py

```

### 3. Verify passing Conditions

Execute the test file and verify if all the tests are passing properly with passing conditions.  
[Terminal 3]
```bash
source /opt/ros/noetic/setup.bash
cd ~/simulation_ws && catkin_make && source devel/setup.bash
rostest tortoisebot_waypoints waypoints_test.test --reuse-master

```

Expected output:
```bash

[ROSTEST]-----------------------------------------------------------------------

[tortoisebot_waypoints.rosunit-tortoisebot_as_integration_test/test_goal_position][passed]
[tortoisebot_waypoints.rosunit-tortoisebot_as_integration_test/test_yaw_alignment][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0

```

### 4. Verify failing Conditions
The code provides the failing conditions which can be selected by passing ROS parameter `test_pass` with value `false`.  
[Terminal 3]
```bash
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash
rostest tortoisebot_waypoints waypoints_test.test test_pass:=false --reuse-master

```

Expected output:
```bash

[ROSTEST]-----------------------------------------------------------------------

[tortoisebot_waypoints.rosunit-tortoisebot_as_integration_test/test_goal_position][FAILURE]
False is not true
--------------------------------------------------------------------------------

[tortoisebot_waypoints.rosunit-tortoisebot_as_integration_test/test_yaw_alignment][FAILURE]
False is not true
--------------------------------------------------------------------------------

SUMMARY
 * RESULT: FAIL
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 2

```
