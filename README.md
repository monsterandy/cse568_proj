# I. Objective

The objective of this project is to first simulate a robot vacuum cleaner, then build the room map, finally construct the shortest comprehensive motion path to cover the entire room.

# II. Description and Approach

According to the different mobile strategies, the complete coverage path planning can be divided into random movement strategy and non-random movement strategy. The random movement strategy is cleaning robot rotates an angle randomly in meet obstacles, so cleaning robot working efficiency is not high by this way, and which can finish the complete coverage of this work area only work for a long time, and the location and environment modeling also need further exploration. The non-random movement strategy controls the motion path of the cleaning robot by using the known environment information and then using the method of graph traversal to path planning.

Our project will implement the non-random movement strategy with known environment information.

At the beginning, the robot doesn’t know the map of the room, so it will first go around the room and build a map. At this stage, the SLAM module will be implemented.

After getting this abstract map of the room, we will translate the continuous map into the discrete occupancy grid. Based on the occupancy grid, implement the complete converge path planning algorithm to find the global path.

Finally, we will use the the local planner which utilizes sensor data for obstacle avoidance to travel to each checkpoint in the global path. At the same time, the path robot traversed will be visualized in the rviz.

# III. Implementation and Packages

As the common structure of the robot cleaner, we decide to use the Turtlebot 3, which is a mobile ground wheeled robot, to simulate the robot cleaner.

For the construction of the physical room, we will use the gazebo to make a map and use that map to simulate the physical room.

For the sensors, we will simply use the laser sensor to get information about obstacles around the robot because the laser sensor is accurate and enough powerful to build a precise 2D map.

For the map building, we plan to use the Google open-sourced Cartographer system to implement real-time simultaneous localization and mapping (SLAM). If this system does not work well as expected, we will turn to try other packages.

For the complete converge path planning algorithm, we plan to use A* algorithm as a heuristic in the U-turn search algorithm. (haven’t decided yet)

For the local planer to traverse each checkpoint, we will implement Vector Field Histogram (VFH) for obstacle avoidance.

P.S. After finishing the complete converge path planning algorithm, if we still have time, we will try to design our own robot from scratch and substitute the Turtlebot.


**Team Name: All the teams’ grades above are invalid**

**Team Member: zheyuanm, keyanguo, yfeng24**
