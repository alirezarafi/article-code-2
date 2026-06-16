# article-code-2
Journal Article Title: Path Planning and Trajectory Tracking Using Type-2/Type-3 Fuzzy Control and Bio-Inspired Optimization for Autonomous Mobile Robots Journal: Applied Soft Computing Publisher: Elsevier DOI: https://doi.org/10.1016/j.asoc.2026.115622
pid-trajectory-tracking
README - PID Controller with Distance-Based Speed Scheduling and Obstacle Avoidance
This code implements a mobile robot navigation framework that combines the Dragonfly Optimization Algorithm for global path planning with a classical PID controller for heading and speed tracking, plus a fuzzy obstacle-avoidance controller.
Requirements:
MATLAB 2021 or newer
Robotics System Toolbox
Fuzzy Logic Toolbox
Optimization Toolbox
Usage:
Ensure that the environment map file `Obstaculo2.mat` is in the project folder.
To run the simulation, execute the following command in MATLAB:
```matlab
runPIDmodel1()
runPIDmodel2()
```
Overview:
The Dragonfly Algorithm (`DragonflyPathPlanning_DF`) is used for global path planning on an occupancy-map-based environment.
The resulting spline path is optionally refined using a local path optimization routine based on a distance transform map (`localPathOptimization`).
A classic PID controller regulates the robot's heading (ψ) and forward speed along the planned path.
The desired forward speed is scheduled based on distance to the final goal and softly reduced near obstacles using range measurements (front/FL/FR).
A fuzzy obstacle-avoidance controller (`a107.m`) is activated when obstacles are detected within an avoidance band (hysteresis on front distance).
Multiple moving obstacles (vertical, horizontal, and circular motion) are simulated and injected into the runtime occupancy map.
The robot is stopped when a moving obstacle becomes too close and resumes when the distance is safe again.
The simulation computes and prints error metrics for:
heading tracking (MSE, IAE, ISE, MAE, RMSE),
speed tracking (MSE, IAE, ISE, MAE, RMSE),
path length error (planned vs. actual traveled distance).
Several plots are generated, including:
Final path (Dragonfly spline, checkpoints, robot trajectory),
Dragonfly convergence curve (best cost vs. iteration),
Left/right wheel voltages,
Linear speeds (forward, planar magnitude) and yaw rate,
Wheel torques and wheel angular speeds,
Scalar robot speed profile,
Robot heading (yaw), X and Y position vs. time.
All key time-series data and paths are packed into a `res` structure and saved to a MAT file (e.g., `run_PID2distogoalm2.mat`) for further analysis and comparison with other controllers (PID+Fuzzy, FOPID, etc.).
Files:
`runPIDmodel1.m`,`runPIDmodel2.m`: Main function for running the PID-based path tracking + obstacle-avoidance simulation.
`Obstaculo2.mat`: Environment/occupancy map used to build the `binaryOccupancyMap`.
`a107.m`: Fuzzy logic controller for local obstacle avoidance based on range sensor readings.
`DynamicalModel.m`: Robot dynamic model (wheel speeds, torques, body velocities, pose, etc.).
`DrawRobot.m`: Helper function for visualizing robot pose and footprint.
Local functions: `DragonflyPathPlanning_DF`, `fitness_DF`, `localPathOptimization`, `localCost`, `estimateGradient`, `getDistanceToObstacle`.
