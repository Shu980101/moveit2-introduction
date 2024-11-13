# moveit2-introduction

Author: [Huanyu Li](https://github.com/HuanyuL)

This is the docker template for MoveIt2 for UR10e robot tutorial and practice 

## Before build the docker
add this into your `bashrc`
```
if [ -f "/dev_ws/setup.bash" ]; then
    source /dev_ws/setup.bash
fi
```

Update the submodules from this repository.
```
git submodule update --init --recursive
```
## How to build the container
To build the image.
```
.docker/build_image.sh
```
To run the image.
```
.docker/run_user.sh
```
You may need to change the owner of the dev_ws, copy the line showing on the terminal.
```
sudo chown -R [YOUR USER NAME] /dev_ws
```
Start a terminal
```
terminator
```
## Launch moveit interface 
To practice the motion planning in simulation.
```
ros2 launch ur_commander iaac_ur10e.launch.py sim:=true
```
To connect to the real robot at IAAC.
```
ros2 launch ur_commander iaac_ur10e.launch.py sim:=false
```
Apprarently the package support ompl and pilz planning pipeline only, you can use the argument to switch different algorithm.
```
ros2 launch ur_commander iaac_ur10e.launch.py sim:true pipeline:pilz
```