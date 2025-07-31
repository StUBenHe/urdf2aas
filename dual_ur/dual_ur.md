# Dual UR Setup
## Install
### Docker
- Install docker.io
[docker](https://docs.docker.com/engine/install/ubuntu/)
- Install docker-compose
[docker](https://docs.docker.com/engine/install/ubuntu/)
- Transfer docker-compose files
	- Full folder Docker/ursim

### ROS2 Humble
- Desctop full
[ros2HumbleInstall](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Install UR ROS2 packages
[ur\_driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/humble)
[ur\_description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/humble)


### ROS2 workspace dual\_ur
- transfer package onto target system
```
cd current workspace
colcon build --symlink-install
source install/local_setup.bash
```

## Run Simulation
### ROS2
- Terminal 1
```
cd Docker/ursim
docker-compose up
```
### Docker-compose
- Terminal 2 -> terminal where you sourced dual\_ur\_ws
```
cd dual_ur_ws
source install/local_setup.bash
ros2 launch dual_ur jtc.launch.py
```
## External control of ur simulation
- go into simulations with provided links from docker-compose terminal
[ur\_1](http://192.168.88.101:6080/vnc.html?host=192.168.88.101&port=6080)
[ur\_2](http://192.168.88.102:6080/vnc.html?host=192.168.88.102&port=6080)
- Click on "Run Program"
- Click on "File" in top left corner
- Click on "Load Program"
- Chose "ext.urp"
- Follow instructions on display if any appear
- Click Play button in bottom bar
- It should display "control\_mode 1" in the "Variables"

## Test external control mode
- Use the following ROS2 publish once method to publish joints
```
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ["ur_1/elbow_joint","ur_1/shoulder_lift_joint", "ur_1/shoulder_pan_joint", "ur_1/wrist_1_joint", "ur_1/wrist_2_joint", "ur_1/wrist_3_joint","ur_2/elbow_joint","ur_2/shoulder_lift_joint", "ur_2/shoulder_pan_joint", "ur_2/wrist_1_joint", "ur_2/wrist_2_joint", "ur_2/wrist_3_joint"], points:[{positions:[1.57, -1.57, 0.0, 0.0, 1.57, 0.0,1.57, -1.57, 0.0, 0.0, 1.57, 0.0], time_from_start: {sec: 10, nanosec: 0}}]}" --once
```

## Shutdown
### ROS2
- ctrl+c in terminal
- wait until terminal returns
### Docker-compose
- ctrl+c in terminal
- wait until terminal returns
```
docker-compose down
```

## Known Issues
### docker ur simulation not reachable
- shutdown and restart docker-compose
- stop terminal execution with crtl+c
```
docker-compose down
docker-compose up
```
### UR simulation robot jiggl
- The ur robots will jiggle in the rviz and ur simulation view
- This is totaly normal and an issue of ur\_cb3 controller settings
- Maybe we will fix this later
