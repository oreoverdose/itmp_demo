## Dependencies

- ROS Kinetic
- Gazebo 7 Simulator
- gazebo\_ros
- This directory in the Gazebo model path: e.g. `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<catkin_ws>/src`

## Usage

Launch the simulation with `roslaunch itmp_demo test.launch`

Move the pioneer model by publishing to the `/cmd_vel` topic. 
(an easy way to do this is with the `teleop_twist` package: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

Lift an object with `rosrun itmp_demo control_lift.py pick_up`

Lower an object with `rosrun itmp_demo control_lift.py set_down`
