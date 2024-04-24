[ROS2 Galactic project]

Follow the instructions below to run the code

                
1. Run the following command to launch the turtlebot3 in maze world.
	ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

2. For teleoperation run the following standard command
	ros2 run turtlebot3_teleop teleop_keyboard  

3. For obstacle avoidance script. 
	ros2 run robot_learning test.py (if this didnt work!!!)
        (You may first enter the directory where the script is located)
        > chmod +x test.py
        > ./test.py


