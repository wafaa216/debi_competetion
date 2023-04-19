
installations : 
we have installed : 
install turtlebot3 pkg from github 
install turtlebot3_simulations from github

1. use debi_ws or copy the pkgs inside it into your workspace  
2. catkin_make
3. roslaunch turtlebot3_gazebo turtlebot3_playground.launch 
	3.1. add the balls in the gazebo world through Gazebo GUI (balls models ar in model_editor_models folder )
4. roslaunch turtlebot3_navigation turtlebot3_navigation.launch 
	4.1. make 2d pose estimation 
5. rosrun dev_pkg get_object_pose_node
6. rosrun dev_pkg send_goal 

Notes : 
- the node : "get_object_pose_node" is  responible to get the balls positions from gazebo and publish thier positions to two topics (/obj1_pose and /obj2_pose )
- the node : " send_goal " is responsible to subscribe to the topics (/obj1_pose and /obj2_pose ) and send the goals to the robot to move towards them
- the robot moves before the ball then it pushes it to the other side 


plan : 
- pushing the ball will be better through giving more accurate series of goals
- code will be integrated to the camera nodes 
- will add the restrictions for the robot to not to pass the red line to the other side 
- add the manipulator to the simulation
