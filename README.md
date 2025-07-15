Use it with
[mrs_octomap_mapping_planning](https://github.com/ctu-mrs/mrs_octomap_mapping_planning/tree/master)
in your workspace. In addition checkout the submodule mrs_octomap_planner to rbl_replanner
branch
to send the robot to the goal first `rosservice call /uav1/octomap_planner/goal (specify goal)`
then `rosservice call /uav1/rbl_controller/activation` 

# To-Do List

- [~] Fix active waypoint for the rbl
- [x] Add reflective markers in simulator
- [ ] Make the replanner team-aware
- [x] Speed up computation of the cell and centroid
- [ ] filter the robot pointcloud for the path planner they should not be considered as obstacles 
- [~] Update service call
- [ ] Adjust parameters map. Robots remain in the map too much 
- [ ] Fix The replanner, sometimes it can be chattering, frequency of replanner probably is too high
- [ ] long-term upgrade to ROS2
- [ ] maybe FlightForge simulator 
- [ ] create image for robofly. to add option for communication between robots.
