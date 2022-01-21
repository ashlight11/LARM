# Branch challenge 3 dedicated to Final Challenge : Bottle Detection, Autonomous Navigation and Goal Targeting

## This repository contains four programs in the form of four ROS launch files.
There are located in the package "grp-poire", in the folder "launch". 
Their purposes are :
  - "challenge3_IRL.launch" launches appropriate nodes to process rgb and depth camera infos, in order to map as well as publish markers corresponding to Nuka Cola bottles found. Usage : 
 > roslaunch grp-poire challenge2.launch rviz:=\<boolean>
  
  The "rviz" parameter is required. If set to true, it launches a visualization with rviz tool in a specific config rviz file. The Rviz file can be found in the "grp-poire" package under the "rviz" folder.
  
## Image Processing with OpenCV : 
- To detect *Black Bottles* : We trained a Haar Cascade, a concept created by Paul Viola and Michael Jones in their publication, "Rapid Object Detection using a Boosted Cascade of Simple Features" in 2001. 
We used a set of 241 positive samples and 642 negative samples. The model was trained with "Haar features" and not Local Binary Patterns. It was trained for 13 stages and it took approximately 36 hours on a 6GB-RAM 4-core PC (One, then two core used for the training). 

- To detect *Orange Bottles* : We use HSV analysis to create a bit mask that will keep only the bright orange pixels. 

Bottles are marked with a green cube. 
  
## Please report any mishaps. 

