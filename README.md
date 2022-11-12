# robotx-2022-launch-machine

## Usage

1. Pull docker and Clone the repo
```
git clone -- recursive https://github.com/yimlaisum2014/robotx2022_launch_machine.git
docker pull yimlaisum2014/robotx2022:shoot
```

2. Enter docker

```
cd robotx2022_launch_machine/Docker
source docker_run.sh
```

If you already run the docker images,
```
source docker_join.sh
```

3. Catkin make the workspace and source environment
```
cd ../$USER/robotx2022_launch_machine
source catkin_make.sh
source enviroment.sh
```
P.S you can set the ROS_IP and ROS_MASTER_IP by typing
``` source environment.sh 10.42.0.2 10.42.0.2```

3. Start the procman 
```
source start_shoot.sh
```

Here is 5 launch
- roscore
- rosserials : run the rosserials script to communcate with arduino by serial
- arm_control : setup the Vx300s allow the user can control
- arm_service : run some serive code allow the user to call
    - go_shoot : position for shooting 
    - go_scan : position for HSI scanning 
    - go_sleep : robot_arm sleep postion
    - go_arm_sleep : short-version Vx300s (adjusting wrist-angle)
- arm_rest : When the all tasks are finish, run this script allow the robot to rest


## Other feature
### 1. single control commend
you could use this command to control single joint of the robot arm.
elbow : (up) 1.3 ~ 1.6 (down)
shoulder (up) -1.6 ~ -1.87 (down)
```
roslaunch arm_control tune_arm.launch set_joint_name:=shoulder set_joint_position:=-1.87
```

## Backup
When procman didn't work

key launch file as following
- roslaunch shoot_control start_rosserial.launch
- roslaunch arm_control shoot_arm.launch
- roslaunch arm_control arm_service.launch
- roslaunch arm_control arm_rest.launch
