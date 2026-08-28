# Lab 1 - Introduction to Robot Control and Dead Reckoning

## Summary

This lab will introduce you to the basic of controlling the robot, as well as familiarizing yourself with robot coordinate frames and performing dead-reckoning.

## Part 1 - Controlling the Robot

First, you will familiarize yourself with controlling a robot. The robot (R2D2) is a differential-drive robot. This means that it has two wheels that can move independently of each other (think of two wheels along the same axis with each connected to an independent motor). One cool aspect of differential drive robots is that compared with vehicles like cars (which, if you are interested, are modeled as Ackermann drive), a differential drive robot can actually rotate in place.

We provide you with a simplified robot simulation that will be used throughout this course. The robot is the famous R2D2. While we will not use Robot Operating System (ROS) for this class (ROS is the typical software stack used in robotics), the interface for controlling R2D2 will be similar to ROS's way of controlling differential drive robots. In particular, you specify a linear velocity (forward/backward) and an angular velocity (around z-axis, i.e. yaw).

If you have an R2D2 object (call it `robot`) in our Python interface, you can send a command with the following (within the implementation object):
```
self.sendCommand(linear, angular)
```
where `linear` and `angular` are floats specifying desired linear and angular velocities, respectively.
The units are in m/s and rad/s, respectively.

### What you need to do:
You need to fill out the controlLoop function so that the robot attempts to perform a square path.

The length of each edge should be the value of `self._segmentDistance` and the time taken in each "part" of the path (e.g., times when robot is moving straight or rotating) is `self._stateTime`. The rotation amount for the corner is `self._segmentRotation`. For more details, look at the code in `implementation.py`. You should have 7 motion segments (4 linear segments and 3 rotate segments).

You will note that even though you can precisely calculate out the values needed, the robot won't achieve a perfect square. Think about why this might be?

## Part 2 - Dead Reckoning

You are then to perform dead reckoning using simulated linear and angular velocity measurements from the robot.

### What you need to do:
Implement `skew` which returns a 3x3 matrix which is skew symmetric.
The function takes vector `v` as argument and returns a matrix, call it `M`. The matrix is constructed such that multiplying a vector $u$ by this matrix $M$ has the same effect as performing a cross product between $v$ and $u$, i.e., $Mu = v \times u$.

Implement `rodrigues` which takes as input an angle-axis rotation vector `omega` and the `deltaTime`, and then returns a 3x3 rotation matrix corresponding to rotating by `omega` for `deltaTime` seconds.

Implement the core part of `measureOdometry`, which computes the new odometric position and rotation. In that function, we already provide you with the (noisy) robot frame velocity and angular velocity, as well as the prior tick's odometric position and rotation. You are to write the code that integrates the velocities to compute the new position and rotation. Use simple first-order Euler integration to perform integration.
