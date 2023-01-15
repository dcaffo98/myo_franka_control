# myo_franka_control
A ros package for intuitive control of the Franka Emika manipulator using the Myo armband.
\
\
![advanced_control_compressed_a_lot_2](https://user-images.githubusercontent.com/54015844/212556460-6b8725cf-bd62-4b87-bb70-384acb106415.gif)

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
![ml_gripper_2x_cropped_compressed](https://user-images.githubusercontent.com/54015844/212556767-f156b0e8-fc7a-420e-b175-e6df68c26cc8.gif)
![ml_control_compressed](https://user-images.githubusercontent.com/54015844/212556654-1ef5cd8b-f1c2-4f2d-bdcc-52a5c53c31f7.gif)
