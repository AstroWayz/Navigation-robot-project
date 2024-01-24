Navigation Robot Project
This project involves building a simulation for a robot capable of navigating a maze filled with obstacles and dead ends, utilizing a navigation algorithm.

Getting Started
To run the project, follow these steps:

Download Gazebo for ROS Melodic version.

Navigate to the workspace folder.

Open a terminal and execute the following command to set up the ROS Melodic environment:

bash
Copy code
source /opt/ros/melodic/setup.bash
Build the project using the following commands:

Copy code
catkin build
Source the setup.bash file:

bash
Copy code
source devel/setup.bash
Launch the mapping component by running:

Copy code
roslaunch commander mapping.launch
Open another terminal and source the setup.bash file:

bash
Copy code
source devel/setup.bash
Stay in the second terminal and execute the following command to visualize the navigation robot:

Copy code
rosrun commander navigation_robot_viz
Now the simulation should be running, allowing the robot to navigate through the maze using the specified navigation algorithm.
