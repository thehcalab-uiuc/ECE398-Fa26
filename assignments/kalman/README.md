# Lab 5 - Introduction to Robot Localization with Kalman Filters

In this lab assignment, you will apply what you've learned about probability and filtering to implement a Kalman filter to perform state estimation and localization with R2D2 (who is equipped with a compass and GPS).

## EKF Localization & State Estimation - Background
Estimating components of a robot's state (such as its position or heading) is a fundamental task in robotics. Determining where a robot is in its environment is known as localization.

A mobile robot operating outdoors like R2D2 might have access to sensors like a GPS (that provides world-centric coordinates), as well as sensors like a compass (to provide world-centric heading).  However, these sensors may be noisy. For example, a GPS might have an error of several meters, even in reasonably good conditions; similarly, magnetic compass readings can also often be quite noisy. While we won't go into the intracacies of modeling noise in this lab, the noise from these sensors can vary a lot and even be affected by nearby objects in the environment or on the robot (e.g., a robot's motors can significantly affect a magnetic compass, as can nearby cars).

To deal with estimating state in the presence of possible noise, Kalman filters are often the "standard" approach. In this lab, you'll develop an Extended Kalman Filter (EKF) that estimates R2D2's state from its controls and sensor measurements (where we use a noisy GPS and compass as our sensors).

### System Model

We will assume R2D2 is modeled by a simple vehicle model:
$$
\dot{\mathbf{x}} = \begin{bmatrix}\dot{p_x} \\ \dot{p_y} \\ \dot{\theta_z} \end{bmatrix} =\begin{bmatrix}-v\sin(\theta_z) \\v\cos(\theta_z) \\ \omega \end{bmatrix}
$$
where $p_x$ is the $x$ position of the robot, $p_y$ is the $y$ position of the robot $\theta_z$ is the yaw of the robot, and $\mathbf{u} = [v, \omega]^T$ is the control input ($v$ is linear forward/backward velocity and $\omega$ is angular yaw velocity).

This equation can also be represented in discrete form (which we'll use for the actual EKF implementation), as:
$$
\mathbf{x}^{t} = \begin{bmatrix}p^{t}_x \\ p^{t}_y \\ \theta^{t}_z \end{bmatrix} = f(\mathbf{x}^{t-1}, \mathbf{u}^{t-1})=\begin{bmatrix}p^{t-1}_x- (\Delta t)(v\sin \theta^{t-1}_z) \\ p^{t-1}_y + (\Delta t)(v\cos \theta^{t-1}_z) \\ \theta^{t-1}_z + (\Delta t)\omega \end{bmatrix}
$$
where $\Delta t$ is our timestep (delta time).

Note that the equation $f(\mathbf{x}, \mathbf{u})$ is non-linear. Thus, we cannot directly use a "normal" Kalman Filter, but we can use an Extended Kalman Filter (EKF)!

We will assume R2D2 has a GPS sensor and a compass, where the GPS provides $x,y$ coordinates of the robot in a global (world) frame and the compass provides the heading of the robot in the global (world) frame.

Thus, we can define a measurement function:
$$
\mathbf{z}^{t} = h(\mathbf{x}^t)=\begin{bmatrix}p^{t}_x \\ p^{t}_y \\ \theta^{t}_z \end{bmatrix}=\mathbf{x}^t
$$

Note: our sensing scheme uses a simplified model of a GPS/magnetic compass. In reality, a GPS provides latitude and longitude, which are angles measured from the center of the Earth rather than providing 2D coordinates on the surface of the Earth as is provided in this lab (the coordinates we use in this lab are similar to UTM coordinates that can be derived from GPS latitude and longitude). Similarly, magnetic compasses can provide heading derived via the magnetic field measurements, but often additional processing is required due to effects like magnetic declination.

### The Extended Kalman Filter

The Extended Kalman Filter (EKF) assumes we have system ($f$) and measurement ($h$) functions like described above. For the EKF in particular, these functions do not have to be linear, whereas for a "normal" KF they need to be linear. More precisely, the EKF assumes:
$$
\mathbf{x}^{t} = f(\mathbf{x}^{t-1}, \mathbf{u}^{t-1}) + \mathbf{n}^{t}_\text{sys}
\\
\mathbf{z}^{t} = h(\mathbf{x}^{t}) + \mathbf{n}^{t}_\text{meas}
$$
where $\mathbf{n}^{t}_\text{sys} \sim N(0,Q^t)$ and $\mathbf{n}^{t}_\text{meas} \sim N(0,R^t)$. $Q^t$ and $R^t$ are the covariance matrices of our Gaussian system noise and Gaussian measurement noise (respectively).

To perform state estimation with a Kalman filter, the filter performs two key steps: the predict step and the update step. The predict step uses the robot's system model ($f$) to estimate how the system evolves given the current state and the input (without considering measurements), whereas the update step then updates the predicted state to incorporate information from sensor measurements. The Kalman filter models our state probabilistically as being Gaussian centered around a mean state and having a covariance matrix that quantifies how uncertain we are in our estimate.

For the following derivations, I denote the state $\mathbf{x}^{t|t-1}$ to represent the estimated mean state at $t$, when including only measurements up to time $t-1$ (once we incorporate the measurement at $t$, it is denoted $\mathbf{x}^{t|t}$). Similarly, $\Sigma^{t|t-1}$ will denote the covariance in the state estimate at $t$ if we only include measurements up to time $t-1$, while $\Sigma^{t|t}$ is the covariance after we've updated to include the measurement at $t$.

The Extended Kalman Filter does not assume linear $f$ or $h$, and to perform the predict and update steps we need the following Jacobians:
$$
F^t = \frac{\partial f}{\partial \mathbf{x}}|_{\mathbf{x}^{t-1|t-1},\mathbf{u}^{t-1}}
$$
and
$$
H^t = \frac{\partial h}{\partial \mathbf{x}}|_{\mathbf{x}^{t|t-1}}
$$

#### Predict Step
The predict step will use the system model (and the control) to predict the evolution of the state. Specifically it yields the predicted state and covariance:
$$
\mathbf{x}^{t|t-1} = f(\mathbf{x}^{t-1|t-1}, \mathbf{u}^{t-1})
$$
$$
\mathbf{\Sigma}^{t|t-1} = F^t \Sigma^{t-1|t-1} (F^t)^T + Q^t
$$

#### Update Step
The update step will use the measurement to correct our predictions.

First, let's compute the residual of the measurement:
$$
y^{t} = \mathbf{z}^{t} - h(\mathbf{x}^{t|t-1})
$$

Now, we calculate a matrix known as the *Kalman Gain*:
$$
K^t = \Sigma^{t|t-1} (H^t)^{T} [H^t \Sigma^{t|t-1} (H^t)^T + R^t]^{-1}
$$
The Kalman Gain can be thought of as a scaling that modulates the amount that our residual will affect our updated state.

Now, we can calculate the updated estimates for our state and covariance as
$$
\mathbf{x}^{t|t} = \mathbf{x}^{t|t-1} + K^t y^t
$$
and
$$
\Sigma^{t|t} = (I - K^{t} H^{t}) P^{t|t-1}
$$


## What You Will Implement
You will implement the Extended Kalman Filter localization system to help R2D2 know where he is and which direction he is pointing when provided noisy sensor measurements from the GPS and compass. 

You should first derive the Jacobians from the system and measurement equations, and then implement the algorithm in the provided `implementation.py` by using the provided utility functions and filling out the TODOs.
Specifically you will implement:
- `kalmanPredict` - you will implement this function that performs the prediction step of the EKF
- `kalmanUpdate` - you will implement this function that performs the update step of the EKF
- `onTick` - you'll modify this function to call the predict and update functions appropriately

To run your implementation, use:
```
python3 assignments/kalman/main.py
```