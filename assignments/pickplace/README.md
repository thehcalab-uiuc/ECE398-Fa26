# Lab 6 - Logistic Classification and Pick n Place

In this lab assignment, you will apply what you learned about learning/inferencing to perform classification of objects for a pick and place task with a robot manipulator.

The manipulator is intended to place a provided object in the correct bin; it will be presented either a mug or a LEGO brick. It should decide what class the object belongs to (mug or LEGO) and then place it into a specific bin depending on what class the object was. To do this, we will use a *logistic regression classifier* that allows us to learn from data how to decide which class the object belongs to.

## Logistic Regression & Gradient Descent - Background

Logistic regression is a common way of performing the task of binary classification -- that is, deciding which of two classes a given input belongs to. Specifically, logistic regression can be thought of as dividing the input data into two classes using a linear decision boundary.

Data is assumed to be able to be divided into two classes: $0$ and $1$. The logistic regression classifier takes as input a vector of features $\mathbf{x}$ and is parameterized by a weight vector $\mathbf{w}$ and bias $b$. The classifier produces an output $f(\mathbf{x})=\sigma(\mathbf{w}^T \mathbf{x}+b)=p \in [0,1]$, where $p$ can be thought of as the probability that the classifier thinks that the provided $\mathbf{x}$ belongs to class $1$ (so the probability it belongs to class $0$ is $1-p$). The function $\sigma(z) = \frac{1}{1+e^{-x}}$ is known as the *sigmoid* function (or the *logistic* function), and is very common in fields like deep learning.

For our implementation, we will assume that $\mathbf{x}$ is a vector consisting of RGB pixels (with values in $[0,1]$) from a flattened input image from a camera. We also append a $1$ to the end of $\mathbf{x}$; this allows us to use the form $f(\mathbf{x})=\sigma(\mathbf{w}^T \mathbf{x})$ rather than $f(\mathbf{x})=\sigma(\mathbf{w}^T \mathbf{x}+b)$, which makes it simpler/cleaner to work with. If we assume our images are of resolution $H \times W$, our images correspond to $M=H \cdot W \cdot 3$ floating point values (the 3 is due to images being RGB). Similarly, we have that $\mathbf{x} \in [0,1]^{M+1}$ and $\mathbf{w} \in \mathbb{R}^{M+1}$.

Note that the prior description of logistic regression can easily be vectorized to perform inference on multiple inputs at the same time. If we have $N$ inputs $\mathbf{x}_1, ..., \mathbf{x}_N$, then we can construct $X \in [0,1]^{N \times (M+1)}$ whose rows are the $\mathbf{x}_1, ..., \mathbf{x}_N$ and we can produce a prediction vector $\mathbf{p} \in [0,1]^{N}$ representing our $N$ predictions as $\mathbf{p}=\sigma(\mathbf{w}^T \cdot X^T)$.

The aforementioned formulation is useful; however, one clear question remains: how do we actually calculate $\mathbf{w}$? The answer is by *training* the classifier!

Assume the probability that a given $\mathbf{x}$ belongs to class $y \in \{0, 1\}$ is given by the following:
$$
p(y|\mathbf{x}) = P(Y=y|X=\mathbf{x}) = \sigma(\mathbf{w}^T\mathbf{x})^y [1-\sigma(\mathbf{w}^T\mathbf{x})]^{1-y}
$$
The exponent of $y$ might be a little confusing, but remember that $y$ must be either 0 or 1 and we're taking a product, so it basically works like a "selector" between the two terms. If $y=1$ we get $p(y|\mathbf{x}) = P(Y=1|X=\mathbf{x}) = \sigma(\mathbf{w}^T\mathbf{x})$ and if $y=0$ we get $p(y|\mathbf{x}) = P(Y=0|X=\mathbf{x}) = 1-\sigma(\mathbf{w}^T\mathbf{x})$, which perfectly matches the probability interpretation of logistic regression discussed previously.

Now, let's assume we have $N$ data points that we wish to fit our model to. Denote a single datapoint's input as $\mathbf{x}_i$ and its desired output (0 or 1) as $y_i$. Let's also assume that each datapoint is independent of the others, so we can write:
$$
p(y_1, ...y_N | \mathbf{x}_1,...,\mathbf{x}_N) = p(y_1 | \mathbf{x}_1) \ ... \ p(y_N | \mathbf{x}_N) = \prod_{i \in \{1, ..., N\}} p(y_i | \mathbf{x}_i) = \prod_{i \in \{1, ..., N\}} \sigma(\mathbf{w}^T\mathbf{x}_i)^{y_i} [1-\sigma(\mathbf{w}^T\mathbf{x}_i)]^{1-y_i}
$$
We'd like to ultimately pick the $\mathbf{w}$ that maximizes the probability of what we observed, so we want to compute $\max_{\mathbf{w}} p(y_1, ...y_N | \mathbf{x}_1,...,\mathbf{x}_N)$ (i.e., we want to maximize the likelihood). However, it's kind of annoying to deal with the product, so instead we use a classic trick, which is minimize the negative log-likelihood:
$$
\begin{align}
\min_{\mathbf{w}} \{-\ln p(y_1, ...y_N | \mathbf{x}_1,...,\mathbf{x}_N)\} &= \min_{\mathbf{w}} \{-\ln \prod_{i \in \{1, ..., N\}} \sigma(\mathbf{w}^T\mathbf{x}_i)^{y_i} [1-\sigma(\mathbf{w}^T\mathbf{x}_i)]^{1-y_i} \} \\
&=\min_{\mathbf{w}} \{-\sum_{i \in \{1, ..., N\}} (y_i \ \ln[\sigma(\mathbf{w}^T\mathbf{x}_i)] + (1-y_i) \ln[1-\sigma(\mathbf{w}^T\mathbf{x}_i)])\}
\end{align}
$$

If we denote $p_i = \sigma(\mathbf{w}^T\mathbf{x}_i)$, then the final sum to be minimized can be written $-\sum_{i \in \{1, ..., N\}}[y_i  \ \ln p_i + (1-y_i) \ln (1-p_i)]$. This form is often known as the cross-entropy loss function in machine learning.

While some basic machine learning methods (including ones you've seen before, like linear least squares regression) have closed form solutions, logistic regression in general doesn't have a closed form expression for its solution to the minimization. However, we note that the loss function is differentiable with respect to $\mathbf{w}$ and so we can use an approach known as gradient descent.

Let's denote $L_{CE}(\mathbf{w})=-\sum_{i \in \{1, ..., N\}} (y_i  \ \ln[\sigma(\mathbf{w}^T\mathbf{x}_i)] + (1-y_i) \ln[1-\sigma(\mathbf{w}^T\mathbf{x}_i)])$ as our loss function. From vector calculus, recall that the gradient $\frac{\partial L_{CE}}{\partial \mathbf{w}}$ will be a vector that points in the direction of *greatest increase* in $L_{CE}$ and $-\frac{\partial L_{CE}}{\partial \mathbf{w}} = -\nabla L_{CE}$ will be a vector that points in the direction of *greatest decrease* in $L_{CE}$. Therefore, if we want to minimize our cross entropy loss, we can use the following update equation:
$$
\mathbf{w}^{t+1}=\mathbf{w}^{t}-\alpha \nabla L_{CE}({\mathbf{w}^{t}})
$$
where $\alpha$ is a small number known as the *learning rate*. This iterative procedure is known as gradient descent. While there are many different variants of gradient descent and different ways to select parameters like the number of update iterations, when to stop, and what learning rate to use, for this lab you'll use a simple fixed learning rate and a fixed number of iterations.

One issue with the prior formulation is that there is nothing that prevents the weights from growing infinitely large; thus, if the data were perfectly linearly separable, you could always lower the loss by increasing the magnitude of $\mathbf{w}$. This can be non-ideal, so we will use an $L2$-regularized version of logistic regression.

Specifically, we will define our loss function as:
$$
L(\mathbf{w}) = \frac{\lambda}{2}\sum_{j=\{1,...,M\}} \mathbf{w}_j^2 - \frac{1}{N} \sum_{i=\{1,...,N\}} [y_i \ln(p_i) + (1-y_i) \ln(1-p_i)]
$$
where $\lambda$ is known as the weight decay constant. Recall that $\mathbf{w}$ is actually $M+1$ dimensional, so we don't apply regularization on the bias weight.

## What You Will Implement
You will implement the logistic regression classifier (using LEGO as the $1$ class and mug as the $0$ class) in the provided `classifier.py` by using the provided utility functions and filling out the TODOs.
Specifically you will implement:
- `sigmoid` - computes the sigmoid of a provided input (should be vectorized)
- `predictSingle` - performs prediction on a single image
- `prepareData` - prepares X and y by processing the provided data
- `computeLoss` - computes the (L2-regularized) logistic regression loss
- `computeGradient` - computes the (L2-regularized) logistic regression gradient
- `train` - trains the logistic regression classifier using gradient descent
- `eval` - evaluates the logistic regression classifier by computing the average binary accuracy over the provided dataset

To train/test the model without running the simulation of the manipulator:
```
python3 assignments/pickplace/train.py
```

To run the full pick and place implementation, use:
```
python3 assignments/pickplace/main.py
```