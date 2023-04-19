# MIE443-Contest1
Explanation of weighted random walk

The algorithm works in the following way, the robot’s state is initially set to ‘SEEK’ and as such, the robot enters the circularScan function.
During the scan, the robot stores distance and yaw values. 
From these values, the code uses the findIdealFunction and newHeading function to point the robot in the ‘ideal’ direction. 
The state of the robot is then set to ‘FREE_ROAM’. 
The code then enters the roaming function which allows the robot to drive around freely unless it encounters a wall or an obstacle. 
If it does, the bumpers are triggered, the robot state is set to ‘BUMPER_HIT’ and the bumperHitMovement function is called. 
Once this function has adjusted the robot away from the hit, the robot state is reset to ‘FREE_ROAM’ and it continues driving around. 
Additionally, when roaming, the robot has a 30% chance of randomly entering the ‘SEEK’ state which will in turn call the circularScan function again. 
These series of steps continue until the timer runs out. 
Note that all robot movement controls are implemented via the trackTurtleBot function. 

To run this code on the turtlebot, execute the following commands in different terminals:
1) roslaunch turtlebot_bringup minimal.launch
2) roslaunch turtlebot_navigation gmapping_demo.launch
3) roslaunch turtlebot_rviz_launchers view_navigation.launch
5) rosrun mie443_contest1 contest1

To run this code on the gazebo simulation, execute the following commands in different terminals:
1) roslaunch mie443_contest1 turtlebot_world.launch world:=1
2) roslaunch turtlebot_gazebo gmapping_demo.launch
3) roslaunch turtlebot_rviz_launchers view_navigation.launch
4) rosrun mie443_contest1 contest1

To capture the results of the mapping, execute the following command in a new terminal:
rosrun map_server map_saver -f /home/turtlebot/
