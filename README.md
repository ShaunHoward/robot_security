# robot_security
The input of n-1 sensors affected by terroristic activities can be accurately predicted by our algorithm.

# installation instructions
- first, install ros indigo (latest turtlebot capable version), make sure to run: echo "source /opt/ros/indigo/setup.bash" >> ~/.bashrc and source the .bashrc
- then run: sudo apt-get install libsdformat1 gazebo2 ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs
- install all the essential python libraries including pip, then run: [sudo] pip install virtualenv
- clone this repository to ~/projects/eecs499/ and cd into robot_security.
- then, run: virtualenv rsvenv
- then: source rsvenv/bin/activate to activate the python venv for installing python libraries w.o affecting global system
- run pip install -r requirements.txt from robot_security directory.
- add to python path in bashrc: PYTHONPATH='/opt/ros/indigo/lib/python2.7/dist-packages:$PYTHONPATH' and source it
- if security issues occur: pip install urllib3[secure]

# to launch turtle bot simulation
- for grid demo run: roslaunch turtlebot_stage turtlebot_in_stage.launch
1. for gazebo: roslaunch turtlebot_gazebo turtlebot_world.launch
2. after gazebo open control turtle bot with keyboard: roslaunch turtlebot_teleop keyboard_teleop.launch
3. run rviz: roslaunch turtlebot_rviz_launchers view_robot.launch
- Note: this will only work if ros indigo and turtlebot simulators are properly installed.

# To launch multi-robot simulation in Gazebo
- sudo apt-get install ros-indigo-robot-localization
- catkin make in ros_ws
- add to ~/.bashrc for python and msg generation:
PYTHONPATH=/opt/ros/indigo/lib/python2.7/dist-packages
export PYTHONPATH="${PYTHONPATH}:/home/shaun/projects/ros_ws/devel/lib/python2.7/dist-packages"
source ~/ros_ws/devel/setup.bash
- source ~/.bashrc
- roslaunch project1 turtlebot_world.launch
- rosrun rviz rviz
- After opening Rviz select the the base_link of whichever robot you would like to view (turtlebot1, turtlebot2, turtlebot3).
- Then add a RobotModel, and change the robot_description to "robot name"/robot_description (e.g. turtlebot1/robot_description) and change the tf_prefix to "robot_name/" (e.g. turtlebot1/)
