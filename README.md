## Dependencies

- ROS Kinetic
- Gazebo 7 Simulator
- gazebo\_ros

## Usage

Launch the simulation with `roslaunch itmp_demo test.launch`

Run a simple demonstration with `rosrun itmp_demo simple_demo.py`

The robot can also be controlled more directly:
    - Move the pioneer model by publishing to the `/cmd_vel` topic (an easy way to do this is with the `teleop_twist` package: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`
    - Lift an object with `rosrun itmp_demo control_lift.py pick_up`
    - Lower an object with `rosrun itmp_demo control_lift.py set_down`
