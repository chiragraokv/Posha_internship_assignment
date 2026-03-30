# Posha_internship_assignment
## packages;
### Piper description
Updated the urdf with static transforms to attac to counter top of kitchen

### piper_automation
Custom pick and drop node for the task, works by parsing waypoints form yaml. The yaml has target coordinates in /world frame, and it sends these goals to move group of piper robot.

### api_python_scripts
has python debug files for 
- printing current gripper position 
- sending goals to move group in wrold coordinates
- scene setup: to set up the kitchen countertop as a collision object.

### custom_movit
- Contains the move group made with movit setup assistant
- this package was made using updated urdf 

## Dependencies

This project uses the [Piper ROS packages](https://github.com/agilexrobotics/piper_ros/tree/humble) for robot description and simulation.
These packages provide the URDF, SRDF, and MoveIt configuration required to simulate and control the Piper robotic arm.