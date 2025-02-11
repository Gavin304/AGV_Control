# AGV Navigation Project

This project leverages ROS 2 (Humble), Slam Toolbox, and Nav2 for autonomous navigation of an AGV.

## Prerequisites
Ensure you have the following installed:
- Slam Toolbox: https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/
- Nav2 Toolbox: https://navigation.ros.org/
- ROS 2 Humble: https://docs.ros.org/en/humble/index.html

## Setup Instructions

1. **Install Dependencies**
   Make sure all required dependencies are installed on your system.

2. **Configure SLAM Toolbox**
   Update the SLAM Toolbox configuration file:
   ```
   cd /opt/ros/humble/share/slam_toolbox/config/
   sudo nano mapper_params_online_async.yaml
   ```
   Adjust parameters as needed.

3. **Launch the AGV Controller**
   Run the following commands to start the AGV controller:
   ```
   ros2 launch agv_launch/launch.py
   ros2 launch lakibeam1 lakibeam1_scan_launch.py
   ```

4. **Run SLAM Program**
   Start the SLAM program by executing the appropriate launch file.

5. **Run Nav2 Program**
   Launch the Nav2 navigation stack for path planning and control.

---
## Video Example


https://github.com/user-attachments/assets/35c5d5d6-f3e1-4459-9b35-9dd9f7ddce0b


