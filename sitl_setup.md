For simulation we will be using ArduPilot SITL and Gazebo in conjuction with ROS2.

Pre reqs: Must have Linux or Windows, if you have a Mac you have to set up a virtual Linux machine (or remote desktop into a Linux/Windows machine).
> [!IMPORTANT]
> Note: Allocate atleast 50 GB on your VM to be able store all the build files / tools.

For Windows install wsl in powershell with: `wsl --install` Then go to the microsoft store and install Ubuntu 22.04 (VERSION IS IMPORTANT). Run Ubuntu and set up.

1. Install ros-humble-desktop for Ubuntu - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
   1. After following each step in the guide, setup the environment with `source /opt/ros/humble/setup.bash` and verify that ros works with the examples in the guide.
2. Make a directory called microros_ws in your home directory and build micro-ros from source - https://micro.ros.org/docs/tutorials/core/first_application_linux
   1. Follow each step in the guide, skipping the step where it talks about a custom app, also ensure that it works with the examples at the end.
3. Build ArduPilot SITL:
   1. clone the ArduPilot repo from https://github.com/ArduPilot/ardupilot.git into your home directory 
   2. `cd ~/ardupilot/`, and run `Tools/environment_install/install-prereqs-ubuntu.sh -y`
   3. reload the path with `. ~/.profile`
   4. then ```run ./waf configure``` and `./waf copter`
4. Make another directory called ardu_ws in the home directory and build ArduPilot ROS capabilites - https://ardupilot.org/dev/docs/ros2.html#ros2
   1. If the build fails when running ```colcon build --packages-up-to ardupilot_dds_tests```, try a clean install of Ubuntu or look up the issue on ArduPilot discussion posts
5. Finally, ensure everything works by running SITL and ROS2 together - https://ardupilot.org/dev/docs/ros2-sitl.html#ros2-sitl
   1. Use the commands for Ardupilot 4.6. For convenience, said commands is included below:
      ```bash
      source /opt/ros/humble/setup.bash
      cd ~/ardu_ws/
      colcon build --packages-up-to ardupilot_sitl
      source install/setup.bash
      ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501
      ```
6. Install Gazebo Harmonic - https://gazebosim.org/docs/harmonic/install_ubuntu
7. Lastly, install the ArduPilot to Gazebo bridge - https://ardupilot.org/dev/docs/ros2-gazebo.html#ros2-gazebo

After following each step carefully you should be able to do:
```bash
cd ~/ardu_ws
source install/setup.bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py
```
And it should bring up Gazebo and start an ArduPilot SITL. You can check ROS topics in another terminal by running ros2 topic list.