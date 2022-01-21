# Branch challenge 3 dedicated to Final Challenge : Bottle Detection, Autonomous Navigation and Goal Targeting

## Explanations and Demo Video : https://youtu.be/28Bt5tDwQIg

## This repository contains four programs in the form of four ROS launch files.
There are located in the package "grp-poire", in the folder "launch". 
Their purposes are :
  - "challenge3_tbot.launch" launches the appropriate node to process rgb and depth camera infos, in order to map (GMapping) as well as publish markers corresponding to Nuka Cola bottles found. It also performs autonomous navigation and obstancle avoidance. Usage : 
 > roslaunch grp-poire challenge3_tbot.launch rviz:=\<boolean> (*default is true*)
 
  - "challenge3_simulation.launch" launches a Gazebo simulation, autonomous navigation and environment mapping with GMapping. Usage : 
 > roslaunch grp-poire challenge3_simulation.launch rviz:=\<boolean> (*default is true*)

Extra launch files : 
- "navigation.launch" launches a Gazebo simulation, as well as Rviz. It implements the move-base module as well as GMapping. Usage : 
 > roslaunch grp-poire navigation.launch

- "navigation_IRL.launch" launches the nodes to set up the Kobuki base, to control the sensors and the move-base node. Also launches Rviz because 2D goals need to be set !! Warning !! It is not fully functional as move-base only sends velocity commands in z rotation. We kept it here to showcase our effort. Usage : 
 > roslaunch grp-poire navigation_IRL.launch
  
If the rviz parameter is set to true, it launches a visualization with rviz tool in a specific config rviz file. The Rviz files can be found in the "grp-poire" package under the "rviz" folder.

## Navigation Model : 
We implemented a ricochet algorithm. Changes were made between challenge 1 and 3. 
The main difference is that we used to have a fixed value for angular rotation. Now, the rotation is adapted according to the width of the object. 
We scan each laser ray that is under a distance threshold, and we increment the rotation angle until the ray no longer corresponds to an obstacle. 
  
## Image Processing with OpenCV : 
- To detect *Black Bottles* : We trained a Haar Cascade, a concept created by Paul Viola and Michael Jones in their publication, "Rapid Object Detection using a Boosted Cascade of Simple Features" in 2001. 
We used a set of 1000 positive samples (made out of 42 distinct ones combined with negative ones) and 642 negative samples. The model was trained with "Haar features" and not Local Binary Patterns. It was trained for 13 stages and it took approximately 36 hours on a 6GB-RAM 4-core PC (One, then two core used for the training). 

- To detect *Orange Bottles* : We use HSV analysis to create a bit mask that will keep only the bright orange pixels. 

Bottles are marked with a green cube. 
  
## Please report any mishaps. 

