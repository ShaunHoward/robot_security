# robot_security
The input of n-1 sensors affected by terroristic activities can be accurately predicted by our algorithm.

# installation instructions
- first, install ros indigo (latest turtlebot capable version), make sure to run: echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc and source the .bashrc
- then run: sudo apt-get install libsdformat1 gazebo2 ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs ros-indigo-rqt ros-indigo-rqt-common-plugins
- install all the essential python libraries including pip, then run: [sudo] pip install virtualenv
- clone this repository to ~/projects/eecs499/ and cd into robot_security
- install any other necessary libraries (to be updated in the future)

# to launch turtle bot simulation
- for grid demo run: roslaunch turtlebot_stage turtlebot_in_stage.launch
1. for gazebo: roslaunch turtlebot_gazebo turtlebot_world.launch
2. after gazebo open control turtle bot with keyboard: roslaunch turtlebot_teleop keyboard_teleop.launch
3. run rviz: roslaunch turtlebot_rviz_launchers view_robot.launch
- Note: this will only work if ros indigo and turtlebot simulators are properly installed.
