# A Example for Calculating Robot Dynamics Using **Pinocchio** Library
Developed by: Xinyang Tian.

Platform: Linux(ubuntu) + [Pinocchio](https://github.com/stack-of-tasks/pinocchio).

In this work, i use **Pinocchio** to verify the dynamics of kuka iiwa7. This library has a lot of functions, such as forward/inverse dynamics and their analytical derivatives, jocobian and its analytical derivatives, etc. The main functions of this script are as follows:
1. Calculating inverse dynamics of the 7-DOF manipulator (kuka iiwa7 as example), include:
    - Total torque of each joint;
    - Gravity contribution ***G(q)***;
    - Inertia Matrix ***M*** of each joint;
    - Coriolis Matrix ***C*** of each joint.
2. Verify the anti-symmetric property of ***dM***/*dt* - 2* ***C***

## Implementation 

First, you should install **Pinocchio** on your system，please follow the procedure described [here](https://stack-of-tasks.github.io/pinocchio/download.html). Then, put the **dy.py** file and **iiwa7_description.urdf** file in the same folder, run the script using *Python*, finally you can get the ***M***, ***C***, and ***G*** with your model.   
- - -
**Note**: if you change the **.urdf** file with your model, you should also change the dimension of ***q***, ***qdot***, and ***qddot*** equal to your model' DOF at the same time. 

## References
[1] Carpentier, J., Saurel, G., Buondonno, G., Mirabel, J., Lamiraux, F., Stasse, O., and Mansard, N, "The Pinocchio C++ library: A fast and flexible implementation of rigid body dynamics algorithms and their analytical derivatives," in 2019 *IEEE/SICE International Symposium on System Integration (SII)*, pp. 614-619.   
[2] R. Featherstone, *Rigid Body Dynamics Algorithms*. Springer, 2008.
