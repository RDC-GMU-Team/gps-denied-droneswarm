For simulation we will be using ardupilot SITL and gazbebo in conjuction wtih ROS2.

Pre reqs: Must have linux or windows, if you have a mac you have to set up a virtual linux machine (or remote desktop into a linux/windows machine)

For windows install wsl in powershell with:
  wsl --install
  then go to the microsoft store and install Ubuntu 22.04 (VERSION IS IMPORTANT)
	Run Ubuntu and set up 

1. Install ros-humble-desktop for Ubuntu - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html
  	i. After following each step in the guide, setup the environment with source /opt/ros/humble/setup.bash and verify that ros works with the examples in the guide.
2. Make a directory called microros_ws in your home directory and build micro-ros from source - https://micro.ros.org/docs/tutorials/core/first_application_linux
    i. Follow each step in the guide, skipping the step where it talks about a custom app, also ensure that it works with the examples at the end.
3. Build ardupilot SITL:
    i. clone the ardupilot repo from https://github.com/ArduPilot/ardupilot.git into your home directory 
    ii. cd ardupilot/, and run Tools/environment_install/install-prereqs-ubuntu.sh -y
    iii. reload the path with . ~/.profile
   	iiii. then run ./waf configure and ./waf copter
5. Make another directory called ardu_ws in the home directory and build ardupilot ROS capabilites - https://ardupilot.org/dev/docs/ros2.html#ros2
    i. If the build fails when running colcon build --packages-up-to ardupilot_dds_tests, try a clean install of Ubuntu or look up the issue on ardupilot discussion posts
6. Finally, ensure everything works with - https://ardupilot.org/dev/docs/ros2-sitl.html#ros2-sitl
    i. I ran into an issue where the file was missing in $(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml, in case you run into the same issue I've added the file to this repo. Just vim dds_xrce_profile.xml then copy and paste the code from the repo.

7. Gazebo - 
  
