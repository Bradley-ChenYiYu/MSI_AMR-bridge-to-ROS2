# Overview
This is a program used to bridge MSI AMR to a ROS2 node.

video is over here:
https://www.youtube.com/@RobotFun-ex3is

- Feature

# How to build

1. Obtaining Source Code

```
$ cd <ROS2_workspace>/src
$ git clone https://github.com/funrobot0804/MSI_AMR-bridge-to-ROS2.git
```

2. Installing Related Packages

```
```

3. Build

```
colcon build --merge-install --packages-select player_bridge --event-handlers console_cohesion+
```

##### run player_bridge
make sure you note book connect to AMR SSID:MSIxxxxxxxxxxxx first and you will get ip 192.168.0.x
the AMR IP will be 192.168.0.2 you can access it via this IP address.

```
$ source install/share/player_bridge/environment/99_player_lib.sh
$ ros2 run player_bridge player_bridge --ros-args -p player_host:=192.168.0.2
```



