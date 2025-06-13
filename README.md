# group7par
PAR projectrepo

cd ~/workspace
colcon build --packages-select vision_to_waypoint_pkg
source install/setup.bash

ros2 run vision_to_waypoint_pkg go_home_node

ros2 run vision_to_waypoint_pkg waypoint_finder

ros2 run vision_to_waypoint_pkg go_to_object
