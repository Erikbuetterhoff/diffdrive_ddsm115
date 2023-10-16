# diffdrive_ddsm115


To run diffdrive on PiBot, follow the commands below.
- `ping pibot` to get his ip address
- `ssh pibot@xxx.xxx.xxx.xxx`
- `cd into workspace`
- `colcon build --symlink-install`
- `source install/setup.bash`
- `ros2 launch diffdrive_ddsm115 diffpibot.launch.py`

In another terminal on the computer with the controller.
- `cd into ros workspace`
- `source install/setup.bash`
- `ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' joy_vel:=/diffbot_base_controller/cmd_vel_unstamped`


# Motors used
[Waveshare DDSM115](https://www.waveshare.com/wiki/DDSM115)

---

This node is designed to provide a ros2_control hardware interface for 2x DDSM115 motors.
It is designed to be used with a `diff_drive_controller` from `ros2_control`.
It is expected to communicate via RS485 serial and to have two DDSM115 motors, using velocity control and position feedback.


It is based on the diffbot example from [ros2_control demos](https://github.com/ros-controls/ros2_control_demos/tree/master/example_2).

For a tutorial on how to develop a hardware interface like this, check out the video below:

https://youtu.be/J02jEKawE5U



## To Do
- [ ] Document changes from earlier versions
- [ ] Clean up remaining connections to original demo code
