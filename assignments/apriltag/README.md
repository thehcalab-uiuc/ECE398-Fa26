# Lab 8 - Vision-Based Localization with April Tags

In this lab, you will implement a vision-based localization system.
Given the robot camera's intrinsics and a detected AprilTag marker, the system will find out where the camera is located relative to the marker. This is done via an analytical solution that can be refined through iterative optimization with the Levenberg-Marquardt algorithm.

## Fiducial Marker Detection
In this lab you'll perform localization using an AprilTag, which is a type of fiducial marker. Fidicual markers are objects that are placed in an environment and used as references for tasks like measurement or localization. You may have seen similar markers before; for example, ArUco markers and QR codes are other technologies that can be used as fidicual markers.

The AprilTag can be easily detected in an image using computer vision and many libraries exist for performing this detection. In general, a detection of an AprilTag will specify the corners of the AprilTag (in a consistent format), as well as the specific ID of the marker (various AprilTag tags exist that each have different patterns).
If you have a calibrated camera and you know the 3D locations of an AprilTag's corners in some coordinate system, as well as the locations of the 2D points of these corners in the image, you can then use the correspondences to calculate where the camera is in the frame of the AprilTag.
While it might seem like it's hard to know the 3D locations of the AprilTag's corners in some coordinate frame, remember that we can define a coordinate frame that is centered at the AprilTag; thus, if we know the dimensions of the tag, we can find out the 3D locations of the corners in the coordinate system of the AprilTag, which can then be used with a calibrated camera and the point correspondences to find the camera's pose relative to the AprilTag.

## Perspective n Point
If we have a calibrated camera, as well as 3D points in some coordinate frame and their 2D points in the camera image, we can calculate the extrinsics of the camera via a procedure known as Perspective n Point (PnP).

Recall that if we have a calibrated camera, we then know the intrinsic parameters of the camera. In this lab, we will assume our matrix $K$ is known and has the form:
$$
K = \begin{bmatrix}f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1\end{bmatrix}
$$

In addition, we assume we have $N$ correspondences (for the case of the AprilTag, $N=4$). Each correspondence has a 2D image point $\mathbf{p}_\text{image}^i \in \mathbb{R}^2$ and its corresponding 3D point $\mathbf{p}_\text{object}^i \in \mathbb{R}^3$ (represented in the object's frame). We seek to use these correspondences to calculate the $6$-DOF pose of the camera, represented as a 3-dimensional translation vector and a 3-dimensional rotation vector.

The PnP problem generally is solved via a two-step procedure. First, an analytical solution is derived. Then, this analytical solution refined through non-linear optimization by minimizing reprojection error (reprojection error is just the error between the detected pixels in the image and the projection of the 3D locations using our estimated 6-DOF camera pose).

Note that the analytical solution can already be quite accurate, but the optimized solution should be even better (we are optimizing the solution given our analytical start point, after all!). You might also wonder why we bother with the analytical solution; couldn't we simply just start off with optimization? This doesn't work in general, and it is really important to provide the optimizer a good initialization point in order to get meaningful outputs. Why might this be?

In practice, often you'll use libraries to solve PnP. For example, OpenCV has a function specifically for this purpose, which we'll use to verify our implementation. If you've implemented the method correctly, it should very closely match OpenCV's implementation.

### Analytical Solution
The first step to solving PnP using a planar fiducial marker (e.g., an AprilTag) involves finding an analytical start point for optimization. To do this, we use a concept known as a *homography*, which is a transformation that relates points on two planes and which can (for our case) be represented by a $3 \times 3$ matrix $H$.

The detection points in the image lie on a plane (the image). Similarly, our AprilTag is a flat marker, so its corners (which are the detection points) also lie on a plane. In other words, each $\mathbf{p}_\text{image}^i=\begin{bmatrix}u_\text{image}^i\\v_\text{image}^i\end{bmatrix} \in \mathbb{R}^2$ is a 2D point on the image plane, and each $\mathbf{p}_\text{object}^i = \begin{bmatrix}x_\text{object}^i\\y_\text{object}^i\\z_\text{object}^i\end{bmatrix} = \begin{bmatrix}x_\text{object}^i\\y_\text{object}^i\\0\end{bmatrix}\in \mathbb{R}^3$ lies on the AprilTag and has $z$ coordinate equal to zero.

Recall that we can use what we know from camera projection to write:
$$
\begin{align}
\begin{bmatrix}u_\text{image}^i\\v_\text{image}^i\\1\end{bmatrix} = \alpha K [R \ \mathbf{t}] \begin{bmatrix}x_\text{object}^i\\y_\text{object}^i\\0\\1\end{bmatrix}
&= \alpha K [\mathbf{r}_1 \ \mathbf{r}_2 \ \mathbf{r}_3 \ \mathbf{t}]\begin{bmatrix}x_\text{object}^i\\y_\text{object}^i\\0\\1\end{bmatrix} \\
&= \alpha K [\mathbf{r}_1 \ \mathbf{r}_2 \ \ \mathbf{t}]\begin{bmatrix}x_\text{object}^i\\y_\text{object}^i\\1\end{bmatrix}\\
&= H\begin{bmatrix}x_\text{object}^i\\y_\text{object}^i\\1\end{bmatrix}
\end{align}
$$
where $H = \alpha K [\mathbf{r}_1 \ \mathbf{r}_2 \ \ \mathbf{t}]$ is our homography and $\alpha$ is some scaling factor (remember that we're working with homogeneous coordinates!).

We can then use the equation $H = \alpha K [\mathbf{r}_1 \ \mathbf{r}_2 \ \ \mathbf{t}]$ to figure out $\mathbf{r}_1$, $\mathbf{r}_2$ and $\mathbf{t}$. We can use the fact that $\mathbf{r}_1$ and $\mathbf{r}_2$ need to be unit vectors to help deal with the scaling factor (note: generally, using information from *both* vectors provides better results, and your implementation should do this). You can find the sign of the scale factor by reasoning about what the sign of the $z$ component of $\mathbf{t}$ should be.
You can find the third rotation vector $\mathbf{r}_3$ using a cross product. Finally, the rotation matrix can be found by concatenating the three rotation vectors together. However, this matrix is not guaranteed to be a valid rotation matrix, so you should use SVD to enforce that it is orthogonal. After correction with SVD, you should also make sure the determinant is $+1$ (after SVD, the determinant must be either $+1$ or $-1$). You can ensure the determinant is $+1$ by multiplying the last *right* singular vector by -1 if $\det(UV^T)=-1$ (where $U$ and $V$ are the matrices of left and right singular vectors).

Once you have found the decomposition, you will have $\mathbf{t} \in \mathbb{R}^3$ and $R \in \mathbb{R}^{3\times 3}$, which represent the *pose of the AprilTag in the camera coordinate frame*.

### Optimization using LM

While the start point found analytically can already be quite accurate (as you'll see from the lab), you can improve results even further using non-linear optimization to reduce reprojection error.

If we define
$$
\mathbf{p}_\text{camera}^i=\begin{bmatrix}x_\text{camera}^i\\y_\text{camera}^i\\z_\text{camera}^i\end{bmatrix}=R \ \mathbf{p}_\text{object}^i + \mathbf{t}
$$
Then, the specific optimization problem is defined as
$$
\min_{\mathbf{r},\mathbf{t}} \sum_{i \in \{1,...,N\}} [([f_x \frac{x_\text{camera}^i}{z_\text{camera}^i}+c_x] - u^i_\text{image})^2 + ([f_y \frac{y_\text{camera}^i}{z_\text{camera}^i}+c_x] - v^i_\text{image})^2]
$$
where we note that the $x_\text{camera}^i$, $y_\text{camera}^i$, and $z_\text{camera}^i$ are functions of $R$ and $\mathbf{t}$, and $\mathbf{r}$ is just the 3-dimensional vector that describes the same rotation as $R$.

The optimization problem clearly has the form of non-linear least squares (NLS) discussed in class, so we can apply the Levenberg-Marquardt (LM) algorithm to solve this optimization problem.

Recall that in the Levenberg-Marquardt algorithm, we calculate our steps using:
$$
(\mathbf{J}^T \mathbf{J})\Delta = \mathbf{J}^T \mathbf{e}
$$
where $\mathbf{e}$ is our prediction residuals and $\mathbf{J}$ is our Jacobian.

There are various strategies for choosing the damping factor $\lambda$ (which in the code will be denoted `L`); in this lab, you should use the strategy given here: https://www.youtube.com/watch?v=UQsOyMj9lnI.
This rule for choosing $\lambda$ involves measuring the change in error that would occur if we take an update step. If the step decreases our error, then we update our solution (i.e., we take the update step) and then multiply $\lambda$ by 0.8. If the step does not decrease our error, then we increasing $\lambda$ by 2 times and do not update the solution (i.e., we don't take the update step).

## What you need to do
You will implement the PnP localization system algorithm in the provided `implementation.py` by using the provided utility functions and filling out the TODOs.
Specifically you will implement:
- `skew` - computes the skew-symmetric matrix from a vector that, when multiplied, corresponds to computing a cross product with the vector.
- `initializationPnP` - computes the analytic initialization (start point) using homography decomposition.
- `optimizePnP` - performs iterative optimization for PnP using the Levenberg-Marquardt algorithm.
- `solvePnP` - solves the PnP algorithm by first finding the analytical start point and then performing iterative optimization.
- `getPoseInAprilTagCoordinates` - given a rotation vector and translation vector of the AprilTag coordinate system in camera coordinates, calculates the rotation vector and translation vector of the camera in the AprilTag coordinate system.

To run your implementation, use:
```
python3 assignments/apriltag/main.py
```

## Sources
The AprilTag is from: ```https://github.com/AprilRobotics/apriltag-imgs/blob/master/tag36h11/tag36_11_00000.png```
