# Lab 7 - MDPs & Value Iteration in a Grid World with R2D2

In this lab assignment, you will apply what you've learned about MDPs and Value Iteration by implementing the value iteration and policy extraction algorithms to help R2D2 plan in a grid world.

## MDPs, Value Iteration, and Policy Extraction - Background
As covered in class, Markov Decision Processes (MDPs) are a common framework for decision making.
MDPs are defined by $(S,A,P,R,\gamma)$, where $S$ is a set of states, $A$ is a set of actions, $P$ is a transition probability $P(s'|s,a)$, $R$ is a reward function, and $\gamma \in [0,1]$ is a discount factor.

The discount factor represents a decay constant that determines how much we should discount the future rewards compared to a current reward; for example, if our agent receives a sequence of rewards $r_1, r_2, r_3, ...$, then the cumulative discounted reward is $r_1 + \gamma r_2 + \gamma^2 r_3 + ...$. Note that for this lab, we use a slightly different convention than that in lecture when calculating discounted reward, as we do not apply the discount factor to the first reward in the sequence. This is very common convention in general reinforcement learning literature, and both the convention here and in lecture are common and valid (but can lead to slightly different numerical results if you're not careful!).

Recall that a *policy* $\pi$ is a mapping from states to actions: $\pi: S \mapsto A$. Thus, you can view $\pi$ as a function $\pi(s)=a$ that tells you what action $a \in A(s)$ to take at each state $s \in S$ (here, I use the notation $A(s)$ to signify that the viable actions might vary depending on the state). Similarly, recall that the *value* $V(s)$ of a given state $s \in S$ represents the potential future rewards from being in that state; more precisely, the value of a given state $V(s)$ when following a policy $\pi$ equals the expected discounted cumulative reward when starting in the state $s$ and taking the actions specified by $\pi$.

As described in lecture, satisfying the *Bellman Equation*:

$$V(s) = \max_{a \in A(s)} \sum_{s' \in S} P(s' | s, a) [R(s,a,s') + \gamma V(s')]$$

is a necessary and sufficient condition for an optimal policy. 

We can actually use this equation to help us get a way to calculate the value of the optimal policy, through a procedure known as *value iteration*. In this algorithm, we first initialize $V^0(s)$ to an arbitrary value (e.g., 0) for all states. Then, the algorithm procedures iteratively. For each iteration $i>0$, we compute:

$$V^{i}(s) \leftarrow \max_{a \in A(s)} \sum_{s' \in S} P(s'|s,a)[R(s,a,s') + \gamma V^{i-1}(s')]$$

for each state to get a new estimate for the value function. Once the values between iterations no longer change, the algorithm has converged. For this lab, your convergence criterion should be $\max_{s \in S}|V^i(s)-V^{i-1}(s)| < \epsilon$, where $\epsilon$ is a convergence threshold.

Once value iteration has converged, we will have the value for the optimal policy $V^{*}(s)$. We can then apply the Bellman Equation again, using a procedure called *policy extraction*, to actually figure out the optimal policy $\pi^{*}$. Specifically, we know that the optimal policy must satisfy the Bellman Equation, so we can find the best action for each state as:

$$\pi^{*}(s) = \text{argmax}_{a \in A(s)} \sum_{s' \in S} P(s' | s, a)[R(s,a,s') + \gamma V^{*}(s')]$$

## Grid World For R2D2
For this lab, we will assume R2D2 is in a grid world. The grid cells in the grid world are of 4 types: *goal* (with reward of 10), *normal* (with a reward of -1), *slippery* (with a reward of -1), and *hazard* (with a reward of -20). In the visualizations, the cells will be colored green, grey, blue, and red, respectively. The actions that can be taken in each cell can vary, and we will assume that the *hazard* and *goal* cells are terminal (i.e., the robot cannot take further actions once it enters one of these cells). For the remaining cell types, the robot can either move up, down, left, or right (except in the case of border cells, where certain actions might be non-viable).

We assume that the transition probabilities vary for the different cell types. If a cell is *normal*, then we assume that we perfectly follow the command (e.g., if the action is up, there is certainty the robot moves up). If a cell is *slippery*, we assume that there is a probability of $0.5$ that the state transitions as desired, with transitions to other states transitions occurring (uniformly distributed) with the remaining probability. Thus, while *slippery* cells and *normal* cells have the same reward, they have different transition probabilities. How would you expect this to affect the final policy?

Note: the transition probability for slippery cells is reflected in the visualization plots and value iteration procedure, but in the simulation of the robot in PyBullet, the robot will always realize the desired state transitions to make your debugging/inspection of the optimal policy easier. Once you have finished your implementation, feel free to modify `simulate.py` if you are interested to see how imperfectly following commands would look!

Finally, we assume that the reward in a given state depends on the cell type in that state; that is, $R(s,a,s')=R(s)=\begin{cases}
    10, & \text{if } s \text{ is goal} \\
    -20, & \text{if } s \text{ is hazard} \\
    -1, & \text{if } s \text{ is normal or slippery} \\
\end{cases}$

## What You Will Implement
You will implement the value iteration algorithm to calculate the values of each state in the grid world, as well as the policy extraction algorithm to extract a policy using the values.
Specifically you will implement:
- `valueIteration` - you will implement this function that performs value iteration and finds the value of each state
- `policyExtraction` - you will implement this function that extracts a policy given calculated values of each state

To run your implementation, use:
```
python3 assignments/valueiter/main.py
```

To run your implementation with different values of the discount factor, use:
```
python3 assignments/valueiter/main.py --discountFactor <DISCOUNT_FACTOR>
```
where <DISCOUNT_FACTOR> is the discount factor you want to use (e.g., 0.9)

You should look at how varying the discount factor changes the final results. How do results change when the discount factor is increased/decreased?