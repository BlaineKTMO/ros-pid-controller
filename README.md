
# Turtlebot PID Controller

A simple, drop-in ROS PID Controller.


## Installation

Install the project by cloning from source.

```bash
  cd catkin_workspace/src
  git clone https://github.com/BlaineKTMO/turtlebot-pid
```
Then catkin_make:
```bash
  cd catkin_workspace && catkin_make
```

Or if using catkin build:
```bash
  catkin build turtlebot-pid
```

    
## Usage

Use this package to create autonomous robot movement on a set path

```bash
roslaunch turtlebot-pid controller.launch
```


## Parameters

```
NODE_NAME: Name of this node
HZ: Loop Rate
PATH_FRAME: Frame_id of path poses
P_GAIN: Proportion factor
D_GAIN: Derivative factor
I_GAIN: Integral factor
PATH_TOPIC: Topic for receiving path data
ODOM_TOPIC: Topic for receiving odometry data
CMD_VEL_TOPIC: Robot velocity topic
SPEED: Target robot velocity

```
## Authors

- [@BlaineKTMO](https://www.github.com/BlaineKTMO)

