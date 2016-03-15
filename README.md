# robot_security
The input of n-1 sensors affected by terroristic activities can be accurately predicted by our algorithm.

# installation instructions
- first, install ros indigo (latest turtlebot capable version)
- then run: sudo apt-get install libsdformat1 gazebo2 ros-indigo-turtlebot ros-indigo-turtlebot-apps ros-indigo-turtlebot-interactions ros-indigo-turtlebot-simulator ros-indigo-kobuki-ftdi ros-indigo-rocon-remocon ros-indigo-rocon-qt-library ros-indigo-ar-track-alvar-msgs
- install all the essential python libraries including pip, then run: [sudo] pip install virtualenv
- clone this repository to ~/projects/eecs499/ and cd into robot_security.
- then, run: virtualenv rsvenv
- then: source rsvenv/bin/activate to activate the python venv for installing python libraries w.o affecting global system
- run pip install -r requirements.txt from robot_security directory.

# to launch turtle bot
- run: roslaunch turtlebot_stage turtlebot_in_stage.launch
- Note: this will only work if ros indigo and turtlebot simulators are properly installed.
