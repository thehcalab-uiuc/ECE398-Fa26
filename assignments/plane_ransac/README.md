# Lab 2 - Ground Plane Fitting and RANSAC

In this lab, you will develop a ground plane estimation algorithm using RANSAC.

## Ground Plane Estimation
One key task in mobile robotics is estimating where the floor of the environment is. For example, if you're moving R2D2 around, you can assume the robot is moving on some kind of ground plane and it is often useful for control/planning to be able to know the equation of the ground plane.

Recall that a plane (in 3-space) can be described as the set of points $(x,y,z)$ that satisfy $ax+by+cz+d=0$ (where $a$, $b$, and $c$ are not all simulatenously zero). You can use methods like linear least squares to estimate the coefficients $a$, $b$, $c$, and $d$.
In our case, we will measure the world-space coordinates of points detected by a depth camera, and use those to fit a ground plane.


## What you need to do
In this lab, you will attempt to compute the ground plane for the environment, which may have a positional offset and an angle offset.

There can be several objects littering the ground, making it hard to directly compute the ground plane as not every point detected by the depth camera will be a ground plane point. In other words, you can consider points not on the floor as outliers, and as discussed in lecture, RANSAC can be extremely effective at dealing with outliers.

### Part 1 - Ground Plane Estimation without Clutter
First, you will implement the plane fitting algorithm without the clutter. For fitting the plane, you should use least squares estimation and the normal equations.
Testing first without clutter allows you to ensure the core plane fitting algorithm is correct.

Specifically, complete the TODOs in the following sections of `implementation.py` (more details are provided as comments/docstrings within the `implementation.py` file):
 - `computePlaneEquation` - should be implemented to compute the plane equation coefficients given an array of points that live in the world coordinate frame
 - `cameraPointsToWorld` - should be implemented to convert points from camera coordinate frame to the world coordinate frame
 - `estimateGroundPlane` - should use the world coordinate frame points to estimate a plane equation by calling the approriate function

To run the program without clutter, use:
```
python3 assignments/plane_ransac/main.py --numCubes 0
```

(Note: for the aforementioned command, and all subsequent commands, I assume you are located in the root ECE398-Fa26 folder.)

The program will print the measured ground plane equation, as well as the groundtruth equation. Without noise, these should match relatively closely.


### Part 2 - Ground Plane Estimation with Clutter
Your implementation for Part 1 should match the groundtruth relatively closely when no clutter is present.
Now, run the program with clutter:
```
python3 assignments/plane_ransac/main.py --numCubes 50
```

You should see that the clutter makes the ground plane estimation significantly less accurate as the ground clutter introduces outliers (you can even set numCubes to a higher value, but it might take longer to run).

As covered in lecture, one very simple but effective way to cope with outliers is the RANSAC (Random Sample Consensus) algorithm.
You should implement the RANSAC algorithm. Several variants and approaches to RANSAC exist; for your implementation, you can adopt the approach of returning the solution that had the largest number of inliers (i.e., you do *not* need to refit on inlier set).

Recall that a point $(x,y,z)$ lies on a plane parameterized by $a$, $b$, $c$, $d$ when the following equation is satisfied: $ax+by+cz+d=0$.
In addition, recall that the distance of a point from the plane is given by $l = \frac{|ax+by+cz+d|}{\sqrt{a^2 + b^2 + c^2}}$. This distance $l$ is zero if the point lies on the plane. We can use this distance to determine whether a point is an outlier; if the distance $l$ exceeds some value $\epsilon_{outlier}$, we say that the point is an outlier.

Complete the TODOs in the following sections of `implementation.py` (more details are provided as comments/docstrings within the `implementation.py` file):
 - `computePlaneEquationRANSAC` - should be implemented to compute the plane equation coefficients given an array of points using RANSAC (with the given number of iterations and outlier epsilon)
 - `estimateGroundPlane` - should be updated to use the RANSAC implementation if the useRANSAC argument is set to true.

Once you've implemented your RANSAC solution, you can run it as follows:
```
python3 assignments/plane_ransac/main.py --numCubes 50 --useRANSAC
```

If RANSAC is implemented correctly, you should see that the deterimental effect of outliers is mitigated substantially by using RANSAC.

You can also sanity-check your RANSAC implementation by changing numCubes to 0 and seeing if it performs similarly to the non-RANSAC version in that case.

You should play around with the value of numCubes. How do the performance of the original and RANSAC methods change as numCubes is varied? Is there a value of numCubes where RANSAC no longer works as well?
