Use it with
[mrs_octomap_mapping_planning](https://github.com/ctu-mrs/mrs_octomap_mapping_planning/tree/master)
in your workspace. In addition checkout the submodule mrs_octomap_planner to rbl_replanner
branch
to send the robot to the goal first `rosservice call /uav1/octomap_planner/goal (specify goal)`
then `rosservice call /uav1/rbl_controller/activation` 

# To-Do List

- [ ] Fix active waypoint for the rbl
- [ ] Add reflective markers in simulator
- [ ] Make the replanner team-aware
- [ ] Speed up computation of the cell and centroid
- [ ] Check if the replanner consider the other robots as obstacle. If so,
      filter them 
- [ ] update service call
- [ ] update README.md
