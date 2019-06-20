# DFF-Manipulation
Dynamic Flex-and-Flip Manipulation of Deformable Linear Objects

![figure1](media/fig1)


## Overview
This repository presents the implementation of flex-and-flip
manipulation tehcnique, suitable for grasping thin, flexible linear
objects lying on a flat surface. For example, we will show how page turning can be be performed with our methods.
Thus, the repository will contain:
1. Code to model deformable linear objects with minimum bending energy curves and two point contacts.
2. Detail fabrication of a soft pneumatic robotic gripper with two fingers suited for the task.
3. And the code to control the UR10 arm on which the gripper can be mounted

Here we assume that an AprilTag is printed on the edge of the page strip to know its position. Of course, this position can be hardcoded.
Modeling is carried out in `MATLAB`, whereas the actual implementation is in Python. 

The following figure shows the process of our flex-and-flip manipulation. The top figure shows when this technique is applied to a page turning task using a two fingered soft robotic hand.

![figure2](media/fig2)


## Usage
Here we expalin different elements of our pipeline.

### Modeling and Manipulating a Linear Deformable Object
The directory `/modeling` contains 1) a `MATLAB` function to generate minimum bending energy curves given the $(x,y)$-coordinates of the right end-point. The left end-point is assumed clamped at the origin. The slope at the left end is set to zero, whereas the right slope is left free. This can be modified in-code.
2) a `MATLAB` scripts that considers a set of feasible right endpoint locations and obtains the minimum bending energy curve for each. Simultaneously, the magnitude of total surface energy as well as the minimum coefficient of friction required to maintain contact \#2 is computed. The following figure was generated using this script.

![figure3](media/fig3)


Knowledge of variation of bending energy and the minimum coefficient of friction can help in planning the path of contact \#2 such as shown in the following figure.

![figure8](media/fig8)
