# Lab 5 - Probabilistic Mapping

In this lab assignment, you will apply what you've learned about probability to develop an occupancy mapping system

## Occupancy Mapping - Background
Occupancy maps are a common representation of the world for mobile robots. These maps essentially describe where in the environment is *occupied* and where is *free*. Such maps can be used for tasks like path planning, as a path can be planned using this map with the goal of passing through only free space.

The general idea of an occupancy map is that it contains a grid of cells, where each cell can be in one of two states. These two states are *occupied* and *free*. A cell being occupied means that there is something in that cell that is detected by a sensor (typically a ranging sensor like a camera or LiDAR). A cell being free means that we did not detect anything in that cell.

The cells do not just have a simple binary state of 1 = occupied and 0 = free. Instead, the mapping is framed probabilistically, where each cell has a probability of being occupied, which we'll denote $P(m_i=1 | z_{1:t}, x_{1:t})$ (or simply $p(m_i| z_{1:t}, x_{1:t})$), where $z_{1:t}$ is the sensor data up until time $t$ and $x_{1:t}$ are the poses up until time $t$.
Therefore, rather than using a naive approach of setting a cell that had a range sensor hit to 1 and setting a cell that was visible and did not have a range sensor hit to 0, we instead perform probabilistic updates of our map.

To perform this probabilistic occupancy mapping, we first assume that each grid cell has a probability of being occupied that is independent of the other grid cells; that is, $p(m_i | z_{1:t}, x_{1:t})$ is independent of $p(m_j | z_{1:t}, x_{1:t})$ for any cells $i \not= j$. This gives us that our entire map $p(m | z_{1:t}, x_{1:t}) = \prod_i p(m_i | z_{1:t}, x_{1:t})$.

Now, we need to write an expression to help us compute $p(m_i | z_{1:t}, x_{1:t})$:
$$
\begin{align}
p(m_i|z_{1:t},x_{1:t})
&= \frac{p(z_t|m_i,z_{1:t-1},x_{1:t}) p(m_i|z_{1:t-1},x_{1:t})}{p(z_t|z_{1:t-1},x_{1:t})} \\
&= \frac{p(z_t|m_i,x_t) p(m_i|z_{1:t-1},x_{1:t})}{p(z_t|z_{1:t-1},x_{1:t})} \\
&= \frac{\frac{p(m_i|z_t,x_t)p(z_t|x_t)}{p(m_i|x_t)} p(m_i|z_{1:t-1},x_{1:t})}{p(z_t|z_{1:t-1},x_{1:t})} \\
&= \frac{p(m_i|z_t,x_t)p(z_t|x_t) p(m_i|z_{1:t-1},x_{1:t-1})}{p(m_i)p(z_t|z_{1:t-1},x_{1:t})}
\end{align}
$$
where in (1) we used Bayes' rule, in (2) we use the fact that we assume $z_t$ does not depend on $z_{1:t-1}$ or $x_{1:t-1}$ (Markov assumption), in (3) we use Bayes rule again, and in (4) we use the fact that $p(m_i)$ should not depend on the robot's pose at time $t$.

Now, we can write the ratio
$$
\begin{align}
\frac{p(m_i|z_{1:t},x_{1:t})}{p(\lnot m_i|z_{1:t},x_{1:t})}
&= \frac{\frac{p(m_i|z_t,x_t)p(z_t|x_t) p(m_i|z_{1:t-1},x_{1:t-1})}{p(m_i)p(z_t|z_{1:t-1},x_{1:t})}}{\frac{p(\lnot m_i|z_t,x_t)p(z_t|x_t) p(\lnot m_i|z_{1:t-1},x_{1:t-1})}{p(\lnot m_i)p(z_t|z_{1:t-1},x_{1:t})}} \\
&= \frac{\frac{p(m_i|z_t,x_t) p(m_i|z_{1:t-1},x_{1:t-1})}{p(m_i)}}{\frac{p(\lnot m_i|z_t,x_t) p(\lnot m_i|z_{1:t-1},x_{1:t-1})}{p(\lnot m_i)}} \\
&= \frac{\frac{p(m_i|z_t,x_t) p(m_i|z_{1:t-1},x_{1:t-1})}{p(m_i)}}{\frac{(1-p(m_i|z_t,x_t)) (1-p(m_i|z_{1:t-1},x_{1:t-1}))}{(1-p(m_i))}} \\
&= \underbrace{\left(\frac{p(m_i|z_t,x_t)}{1-p(m_i|z_t,x_t)}\right)}_{\text{observation}} \underbrace{\left(\frac{p(m_i|z_{1:t-1},x_{1:t-1})}{1-p(m_i|z_{1:t-1},x_{1:t-1})}\right)}_{\text{recursive}} \underbrace{\left(\frac{1-p(m_i)}{p(m_i)}\right)}_{\text{prior}}
\end{align}
$$

Now, we can use a tool called the *Log-Odds*, which is given by:
$$
\text{LogOdds}(x) = l(x)=\ln\frac{p(x)}{1-p(x)}
$$
Note: you can solve for $p(x)$ to recover the probability if you have the log-odds $l(x)$.

If we write the log-odds of $p(m_i|z_{1:t},x_{1:t})$ and use the factorization given above (with the observation, recursive, and prior terms), we can get:
$$
l(m_i|z_{1:t},x_{1:t}) = \ln\frac{p(m_i|z_{1:t},x_{1:t})}{1-p(m_i|z_{1:t},x_{1:t})}=\ln\frac{p(m_i|z_{1:t},x_{1:t})}{p(\lnot m_i|z_{1:t},x_{1:t})} = \underbrace{l(m_i|z_t,x_t)}_{\text{inverse sensor model}} + \underbrace{l(m_i|z_{1:t-1},x_{1:t-1})}_{\text{previous log-odds}} - \underbrace{l(m_i)}_{\text{prior}}
$$

Thus, if we use log-odds, we can update our map cells in an additive fashion. Essentially, we have that the log-odds of the map cell being occupied at the current timestep is equal to the inverse sensor model plus the log-odds for the cell being occupied at the previous time step minus a prior. Once the log-odds have been computed, you can then simply calculate the probability that the cell is occupied from the log-odds.

For this assignment, you will use a very simple inverse sensor model:
$$
l(m_i|z_t,x_t)=\begin{cases}
l_{\text{free}} & \text{if cell was marked as free}\\
l_{\text{occ}} & \text{if cell had range sensor return} \\
0 & \text{otherwise}
\end{cases}
$$
where $l_{\text{free}}$ is the log-odds of $p_{\text{free}}$ and $l_{\text{occ}}$ is the log-odds of $p_{\text{occ}}$.

In our code, $p_{\text{free}}$ is given by ``self._pFree``, $p_{\text{occ}}$ is given by ``self._pOcc``, and $p(m_i)$ is given by ``self._pPrior``.

For more information on this topic (as well as other related topics like Bayesian Filtering), we highly recommend the book "Probabilistic Robotics" (Thrun et al.) and the YouTube videos by Cyrill Stacchnis.

## What You Will Implement
You will implement the occupancy mapping algorithm in the provided `implementation.py` by using the provided utility functions and filling out the TODOs.
Specifically you will implement:
- `probabilityToLogOdds` - converts a numpy array of probabilities to log-odds
- `logOddsToProbability` - converts a numpy array of log-odds to probabilities
- `scanPointsToWorld` - converts points in camera frame to world frame
- `updateOccupancyMap` - updates an occupancy map recursively

To run your implementation, use:
```
python3 assignments/occupancy/main.py
```