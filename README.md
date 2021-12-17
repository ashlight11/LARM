## Branch challenge 1 dedicated to movement and obstacle detection

# This repository contains two programs in the form of two ROS launch files.
There are both located in the package "grp-poire", in the folder "launch". 
Their purposes are :
  - "challenge1_simulation.launch" launches a Gazebo simulation stage, as well as a python script to make the robot move and avoid obstacles. Usage : 
 > roslaunch grp-poire challenge1_simulation.launch rviz:=\<boolean>
   - "challenge1_turtlebot.launch" launches the minimal launch configuration for turtlebot, as well as a python script to make the robot move and avoid obstacles. Usage : 
 > roslaunch grp-poire challenge1_turtlebot.launch rviz:=\<boolean>
  
  In both cases, the "rviz" parameter is required. It should be set to true if a visualization with rviz tool in a specific config rviz file is desired. Rviz can be found in the "grp-poire" package under the "rviz" folder.
  
## Please report any mishaps. 
