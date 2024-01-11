# RobotProgWork1
My first ROS-related project with Python. Uses a LIMO Robot to navigate a simple world and detect potholes.

Installing ROS2 Humble desktop
    https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

Installing LIMO Simulation native
    https://github.com/LCAS/CMP9767_LIMO/wiki/Simulator-Setup

Creating the ROS workspace and clone the repository
    1. Create new workspace(or use an existing workspace.)
        mkdir ros2_ws1
    2. Create 'src' folder
        cd ros2_ws1
        mkdir src
    3. Clone the repository to src folder
        git clone https://github.com/Kanixre/ros2_ws1.git
    4. Build the packege
        cd ~/ros2_ws1
        colcon build --symlink-install
    5. Source the packege
        . install/setup.bash
    6a. Run the Gazebo Simulator for the Pothole simple world
        cd limo_ros2/
        source install/setup.bash
        ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/limo_gazebosim/worlds/potholes_simple.world
    6b. For the Pothole realistic world
        ros2 launch limo_gazebosim limo_gazebo_diff.launch.py world:=src/limo_gazebosim/worlds/potholes.world

In a different terminal, start RVIZ2 graphical visualiser
    rviz2 -d src/limo_gazebosim/rviz/urdf.rviz

ROS2 Commands

To launch all nodes at once
    cd ../ros2_ws1/src/robot_controller_1/launch
    ros2 launch pothole_detect_bot.launch.py

Or to Run the nodes one by one,
    ros2 run robot_controller_1 robot_nav
    ros2 run robot_controller_1 object_detector
    ros2 run robot_controller_1 object_counter

Summary
    1. The robot_nav.py simply moves the robot around the simulated area using a twist command.
    2. The object_detector.py detects potholes in the environment using opencv. It displays a window (either seperate or in rviz - the change cane be made in the code) which shows the potholes and its estimated size from a distance between 0.3 and 0.5 metres.
    3. The object_counter.py reports the number of potholes detected and their locations relative the odom(world). The detector node deeds to be running as the counter depends on the marker info being published from it. It also prints the this information to a text file and saves it to the launch folder. The name can be changed to object_reporter too ;)
    