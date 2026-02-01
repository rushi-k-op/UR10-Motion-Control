UR10 Robot Motion Control with ROS 2 & MATLAB

Project Overview
This project implements a complete motion control system for a Universal Robots UR10 manipulator. It interfaces MATLAB with ROS 2 to perform kinematic modeling, trajectory generation, and execution via both Topic Publishers and Action Clients.

The core challenge involved generating a continuous quintic spline trajectory through multiple waypoints without stopping, overcoming the standard "stop-and-go" motion profile of default trajectory generators.

Key Features
- Inverse Kinematics: Solves for joint angles using a RigidBodyTree model to reach specific Cartesian poses .
- Continuous Trajectory Execution: Custom velocity vector generation to ensure smooth, non-stop motion through intermediate waypoints .
- ROS 2 Action Client: Robust execution using the FollowJointTrajectory action interface with feedback monitoring .
- Real-time Visualization: Monitors joint states via MATLAB subscribers and plots velocity profiles to verify motion continuity.

Tech Stack
- Robot: Universal Robots UR10
- Middleware: ROS 2 (Humble/Foxy)
- Simulation: Gazebo (Headless Mode) & RViz
- Control Logic: MATLAB ROS Toolbox

Results
1. Continuous Motion (Velocity Profile)
The robot executes a path: Home $\to$ Intermediate $\to$ Final.The plot below demonstrates non-zero velocity at the intermediate waypoint ($t=2.5s$), proving the robot did not stop.
2. Action Client Execution
The same trajectory reproduced using the FollowJointTrajectory Action Client, confirming robust control integration.

Installation & Usage

Prerequisites
- MATLAB (R2022b or newer) with ROS Toolbox.
- Ubuntu (22.04) with ROS 2 installed.
- ur_simulation_gazebo package installed in your ROS 2 workspace.

Running the Simulation
1. Start the ROS 2 Simulation (Terminal 1):
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py ur_type:=ur10 gazebo_gui:=false
(Note: gazebo_gui:=false is used for performance optimization)
2. Run the MATLAB Script:
- Open src/ROS2RobotMotionControl.mlx in MATLAB.
- Ensure the helpers folder is added to your MATLAB path.
- Run the sections sequentially to initialize the ROS node, solve IK, and execute trajectories.

Project Structure
- src/: Contains the main MATLAB control script and helper functions for data conversion.
- docs/: Detailed PDF reports explaining the mathematical approach and code implementation.

Author: Rushikesh Rajesh Kaduskar Date: February 2026