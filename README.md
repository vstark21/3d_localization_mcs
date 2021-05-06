# 3D Localization Using Multi Camera System

​	A common problem in the field of autonomous robots is how to obtain the position and orientation of the robots within the environment with sufficient accuracy. Several methods have been developed to carry out this task. The localization methods can be classified into two groups : those that require sensors onboard the robots and those that incorporate sensors within the environment. Although the use of sensors within the environment requires the installation of an infrastructure of sensors and processing nodes, it presents several advantages, it allows reducing the complexity of the electronics onboard the robots and facilitates simultaneous navigation of multiple mobile robots within the same environment without increasing the complexity of the infrastructure. 

​	Moreover, the information obtained from the robots movement is more complete, thereby it is possible to obtain information about the position of all of the robots, facilitating cooperation between them. The sensor system in this work is based on an array of calibrated and synchronized cameras. There are several methods to locate mobile robots using an external camera array. The proposal presented in this paper uses attached artificial landmarks to locate robots and the camera geometry to obtain the positions. It uses a set of calibrated cameras for which pinhole imaging principle applies, placed in fixed positions within the environment to obtain the position of the robots.

## Approach

We consider a multi-camera system with four cameras in it. But the proposed approach can be applied to multi-camera systems with any number of cameras.
The task can be broadly divided into four parts: 

* Locating the robot in each view.
* Removing the effects of distortion.
* Conversion from 2D camera frame(camera plane) to 3D camera frame
* Conversion from Camera frame to World frame.
* Finding the optimal solution.

More details can be found in [report](Exploratory_Project.pdf).