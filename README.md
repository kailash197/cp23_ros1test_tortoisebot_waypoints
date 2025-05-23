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

Execute the test file and verify if all the tests are passing properly with passing conditions.
[Terminal 3]
```bash
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash
rostest tortoisebot_waypoints waypoints_test.test --reuse-master

```

Expected output:
```bash
[ROSTEST]-----------------------------------------------------------------------

[tortoisebot_waypoints.rosunit-tortoisebot_as_integration_test/test_goal_position][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 1
 * ERRORS: 0
 * FAILURES: 0

```

#### 2. Failing Conditions

[Terminal 3]
```bash
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash
rostest tortoisebot_waypoints waypoints_test.test test_pass:=false --reuse-master

```

Expected output:
```bash
[Testcase: testtortoisebot_as_integration_test] ... ok

[ROSTEST]-----------------------------------------------------------------------

[tortoisebot_waypoints.rosunit-tortoisebot_as_integration_test/test_goal_position][FAILURE]
False is not true
--------------------------------------------------------------------------------

SUMMARY
 * RESULT: FAIL
 * TESTS: 1
 * ERRORS: 0
 * FAILURES: 1

```

### 2. Manually start all nodes (without using test launch files)

#### 1. Launch the Waypoints Action Server for ROS1
[Terminal 2]
```bash
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash
rosrun tortoisebot_waypoints tortoisebot_action_server.py

```
Expected Output:
```bash
[INFO] [1748015167.206618, 3772.189000]: Action server started

```

#### 2. Run the passing test
[Terminal 3]
```bash
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash

rosparam set /test_pass true
rosrun tortoisebot_waypoints tortoisebot_as_integration_test.py

```
Expected output:
```bash
[Testcase: test_goal_position] ... ok
-------------------------------------------------------------
SUMMARY:
 * RESULT: SUCCESS
 * TESTS: 1
 * ERRORS: 0 []
 * FAILURES: 0 []

 ```

#### 3.Run the failing test

Restart waypoint action server in step 1.

[Terminal 3]
```bash
source /opt/ros/noetic/setup.bash
source ~/simulation_ws/devel/setup.bash

rosparam set /test_pass false
rosrun tortoisebot_waypoints tortoisebot_as_integration_test.py

```

Expected Output:
```bash
[Testcase: test_goal_position] ... FAILURE!
FAILURE: False is not true
-------------------------------------------------------------
SUMMARY:
 * RESULT: FAIL
 * TESTS: 1
 * ERRORS: 0 []
 * FAILURES: 1 [test_goal_position]

 ```






