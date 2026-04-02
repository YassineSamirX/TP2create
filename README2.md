# DRAW_SQUARE 

Terminal 1 — Le simulateur

source /opt/ros/humble/setup.bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py

Terminal 2 — Le serveur draw_square


source /opt/ros/humble/setup.bash
cd ~/projet/TP2create
colcon build
source install/setup.bash
ros2 run draw_square draw_square_server

Terminal 3 — Envoyer le goal

source /opt/ros/humble/setup.bash
source ~/projet/TP2create/install/setup.bash
ros2 action send_goal /draw_square custom_interfaces/action/DrawSquare "{side_length: 0.5}"



# Boundary_follow 

Terminal 1 : simulator

source /opt/ros/humble/setup.bash
ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py


Terminal 2 :  Le serveur boundary_follow

source /opt/ros/humble/setup.bash
cd ~/projet/TP2create
colcon build --packages-select boundary_follow
source install/setup.bash
ros2 run boundary_follow boundary_follow_server

Terminal 3 : Envoyer le goal

source /opt/ros/humble/setup.bash
source ~/projet/TP2create/install/setup.bash
ros2 action send_goal /boundary_follow custom_interfaces/action/BoundaryFollow "{duration: 30.0}"