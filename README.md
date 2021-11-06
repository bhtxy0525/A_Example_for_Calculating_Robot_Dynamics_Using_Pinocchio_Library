# A Example for Calculating Robot Dynamics Using Pinocchio Library
Developed by: Xinyang Tian.

Platform: Python.

In this work, the dynamic model of a 7-DOF robot manipulator has been established. Here are some notes about this script:

- Using recursive Newton-Euler algorithm to calculate the inverse dynamic model of serial robot manipulator (only rotational joints) with modified DH parameters.
- The parameters of the modified DH coordinates, inertia tensor, centroid coordinates and mass of each link can be found in the ***iiwamodel*** file.
- No external force/torque.

## Implementation 
The code is implemented in MATLAB R2019b. Also, other versions of Matlab are available. Before running this script, pleas enter the input parameters: q (1×7), qd (1×7), and qdd (1×7). Cubic or quintic polynomials are recommended for planning trajectory. As a consequence, the output parameters are seven torques corresponding to seven joints of the robot manipulator.

## References
[1] Q. Zhan, Robotics: Mechanisms Kinematics, Dynamics and Motion Planning, Tsinghua University Press, China, 2019.  
[2] A. De Luca and L. Ferrajoli, "A modified newton-euler method for dynamic computations in robot fault detection and control," 2009 IEEE International Conference on Robotics and Automation, 2009, pp. 3359-3364.
