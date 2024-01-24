# Navigation Robot Project

This project involves building a simulation for a robot capable of navigating a maze filled with obstacles and dead ends, utilizing a navigation algorithm.

## Getting Started

### Prerequisites
- [Gazebo](http://gazebosim.org/) for ROS Melodic version

### Installation Steps

1. **Download Gazebo for ROS Melodic:**
   Download and install Gazebo for ROS Melodic version from the official website.

2. **Navigate to Workspace:**
   Move to the workspace folder of the project.

3. **Set Up ROS Melodic Environment:**
   Open a terminal and execute the following command to set up the ROS Melodic environment:
   ```bash
   source /opt/ros/melodic/setup.bash
   ```

4. **Build the Project:**
   Build the project using the following commands:
   ```bash
   catkin build
   ```

5. **Source the Setup Script:**
   Source the setup.bash file:
   ```bash
   source devel/setup.bash
   ```

6. **Launch Mapping Component:**
   Launch the mapping component by running:
   ```bash
   roslaunch commander mapping.launch
   ```

7. **Visualize Navigation Robot:**
   Open another terminal and source the setup.bash file:
   ```bash
   source devel/setup.bash
   ```

   Stay in the second terminal and execute the following command to visualize the navigation robot:
   ```bash
   rosrun commander navigation_robot_viz
   ```

Now the simulation should be running, allowing the robot to navigate through the maze using the specified navigation algorithm.
