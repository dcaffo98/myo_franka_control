# myo_franka_control
A ros package for intuitive control of the Franka Emika manipulator using the Myo armband.
\
\
![ezgif-2-0d2f8bddbe](https://user-images.githubusercontent.com/54015844/211851424-28c518b8-18df-4d8b-80c9-b553009f16f6.gif)
### How to run
Provided that **you have a Myo armband at hand** and all the [dependencies](https://github.com/dcaffo98/myo_franka_control/blob/master/package.xml#L51) are installed, go to your catkin workspace and run
```
$ catkin build myo_franka_control
$ source ~/catkin_ws/devel/setup.bash
```
Then you have to plug-in the Myo bluetooth dongle and give it the read/write permissions:
```
$ sudo chmod r+w /dev/ttyACM0 # or /dev/ttyACM1, you have to figure it out
```
You're now ready to launch:
```
# simple control -> fast but unreliable because of singularities
$ roslaunch myo_franka_control main.launch cmd_exec_mode:=simple use_caffeine:=true
```
or
```
$ roslaunch myo_franka_control main.launch cmd_exec_mode:=mom use_caffeine:=true
```
for the advanced control loop, which aims to [maximise the measurement of manipulability](https://jhavl.github.io/mmc/). Unfortunately, depending on your machine, it could be very slow.
\
\
***Currently, the user is assumed to hold a position in which the hand of the arm wearing the Myo is facing downward (think as if your hand was the gripper of the manipulator in its initial pose in the gazebo simulation) until the simulation is fully loaded.***
\
\
![ezgif-2-a05ee44d81](https://user-images.githubusercontent.com/54015844/211850722-752c076b-305d-43c0-816c-e14fcef77d95.gif)
![ezgif-2-4643a0f39e](https://user-images.githubusercontent.com/54015844/211850052-4bec1427-a04f-4698-993e-f3f93fa16d9d.gif)
