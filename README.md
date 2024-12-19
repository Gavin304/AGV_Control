Package requirements:
Slam Toolbox
Nav2 toolbox
Ros2 Humble

To run Install all the dependencies
then set the config file for the ros toolbox
cd /opt/ros/humble/share/slam_toolbox/config/
sudo nano mapper_params_online_async.yaml 

launch agv_controller
 ros2 launch agv_launch/launch.py 
 ros2 run lakibeam1 lakibeam1_scan_node 


run slam Program 
run nav2 program
