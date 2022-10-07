## obstacle_avoidance

This project simulates an apartment building delivery robot attempting to get from a starting position to a recipients door.

Key steps:
- Robot is assumed to have access to the building's floorplan.
- A probabilistic roadmap planner is then used to calculate an ideal path to the goal from it's starting point.
- Obstacles are randomly placed in the apartment building to block this path.
- Measurements from Lidar onboard the robot are processed to allow for the use of the Vector Field Histogram (VFH) planning algorithm. This enables the robot to compute obstacle-free steering directions in real-time.
- The robot then attempts to follow the ideal path. If it encounters an obstacle it can use it's lidar measurements and the VFH algorithm to manouevre around the obstacle before returning to the ideal path.

Completed using MATLAB and Mobile Robotics Simulation Toolbox. 