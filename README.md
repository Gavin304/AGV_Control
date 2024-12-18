matlab -softwareopengl
ros2 launch slam_toolbox onli0params_file:=~/ros2_ws/config/my_slam_params.yaml	


cd /opt/ros/humble/share/slam_toolbox/config/
sudo nano mapper_params_online_async.yaml 



 ros2 launch agv_launch/launch.py 

ros2 run js2fork js2fork 
ros2 run forklift forklift 
ros2 run joystick_ros2 joystick_ros2
