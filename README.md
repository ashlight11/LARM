## Branch challenge 1 dedicated to movement and obstacle detection

# This repository contains two programs in the form of two ROS launch files.
There are both located in the package "grp-poire", in the folder "launch". 
Their purposes are :
  - "challenge1_simulation.launch" launches a Gazebo simulation stage, as well as a python script to make the robot move and avoid obstacles. Usage : 
 > roslaunch grp-poire challenge1_simulation.launch rviz:=\<boolean>
   - "challenge1_turtlebot.launch" launches the minimal launch configuration for turtlebot, as well as a python script to make the robot move and avoid obstacles. Usage : 
 > roslaunch grp-poire challenge1_turtlebot.launch rviz:=\<boolean>
  
  In both cases, the "rviz" parameter is required. It should be set to true if a visualization with rviz tool in a specific config rviz file is desired. Rviz files can be found in the "grp-poire" package under the "rviz" folder.
  Please note that only the simulation sends PointCloud2 data to RVIZ. */#Work in Progress*
  
We did not define any exploration method other than bouncing and turning around when facing obstacles. 
In the simulation, we implemented a reverze option : when the robot is too close from an obstacle, we move back quickly (negative x speed). 

Please understand differences between simulation and *in lab* are solely due to a lack of time.  
  
## Please report any mishaps. 
