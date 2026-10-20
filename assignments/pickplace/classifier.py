#
# This is the file you will be modifying.
#


import numpy as np


EPS = 1e-7


class LogisticClassifier:
    def __init__(self, imageWidth=32, imageHeight=32):
        """ Logistic regression classifier
        
        DO NOT MODIFY
        """
        self._imageWidth  = imageWidth
        self._imageHeight = imageHeight
        self._inputDimension = imageWidth * imageHeight * 3 + 1 # adding 1 for the bias

        self.weights = np.random.normal(0.0, 0.01, size=(self._inputDimension))

    def imageToVector(self, image):
        """ Convert an image (H,W) to a flattened vector
        
        We also transform it from its original type (assumed np.uint8 with 0-255) to a float in range 0-1.

        DO NOT MODIFY

        :param image: raw image (H,W,3)
        :returns: image as vector
        """
        if image.shape[0:2] != (self._imageHeight, self._imageWidth):
            raise ValueError("Image is not of the correct size! Should be ({},{})".format(self._imageHeight, self._imageWidth))

        return image.flatten().astype(np.float32) / 255.0
    
    # Your implementation here!
    # -----------------------------------------------------------------------
    def sigmoid(self, x):
        """ Calculate the value of a sigmoid function

        Should be vectorized!
        
        :param x: input (np.array)
        :returns: sigmoid(x)
        """
        # TODO:
        # Implement sigmoid
        return 0.5 * np.ones_like(x)

    def predictSingle(self, image, weights):
        """ Perform prediction on a single image

        Hint: self.imageToVector may be helpful!
        Ensure you prepare the data properly with the bias!
        
        :param image: raw image
        :param weights: weights vector to use for prediction
        :returns: prediction (float), which should be in (0,1) range
        """
        # TODO:
        # Convert image to needed inference format, perform inference, and return a float prediction value
        # Your answer here
        return 0.5

    def prepareData(self, data):
        """ Prepare data by converting data dictionary to matrix of inputs and vector of desired outputs
        
        The X should be the stacked images as vectors, including the bias term 1 (should be last element of each vector).
        The y should be a vector that is 0 for not LEGO (i.e., mug) and 1 for LEGO.
        Have your data be [not LEGO..., LEGO...] (i.e., mug entries are first in lists).

        Hint: self.imageToVector may be helpful!

        :param data: data to use for evaluation. It is expected to be a dictionary, with the following
            - 'mug': list of raw images
            - 'lego': list of raw images
        :returns: tuple for inputs as numpy array (N, len(weights)) and outputs as numpy array (N,)
        """
        # TODO:
        # Prepare the data by creating X, y
        # Your answer here
        return (
             np.zeros((len(data['mug']) + len(data['lego']), 3 * self._imageHeight * self._imageWidth + 1)),
             np.zeros(len(data['mug']) + len(data['lego']))
        )

    def computeLoss(self, X, p, y, weights, weightDecay):
        """ Compute the loss of the logistic regression classifier
        
        The loss function is as follows:
            L(w) = lambda/2 * sum_{j=1,...,M} w_j^2 + sum_{i=1,...,N} 1/N [-(y_i log(p_i) + (1-y_i)log(1-p_i))]
        
        Note that the weight decay is NOT applied to the final weight (the weight corresponding to the bias).
        This function should work with arbitrary N and len(weights). Do not hard-code sizes.

        :param X: input (N, len(weights))
        :param p: computed output (N,)
        :param y: desired output (N,)
        :param weights: weights vector to use (M+1,)
        :param weightDecay: weight decay constant
        :returns: the loss
        """
        # TODO:
        # Compute and return the loss
        # Your answer here
        return 0.0

    def computeGradient(self, X, p, y, weights, weightDecay):
        """ Compute the gradient of the loss of the logistic regression classifier
        
        The loss function is as follows:
            L(w) = lambda/2 * sum_{j=1,...,M} w_j^2 + sum_{i=1,...,N} 1/N [-(y_i log(p_i) + (1-y_i)log(1-p_i))]
        
        This function should return the gradient of that loss with respect to the weights.

        Note that the weight decay is NOT applied to the final weight (the weight corresponding to the bias).
        This function should work with arbitrary N and len(weights). Do not hard-code sizes.
        
        :param X: input (N, len(weights))
        :param p: computed output (N,)
        :param y: desired output (N,)
        :param weights: weights vector to use (M+1,)
        :param weightDecay: weight decay constant
        :returns: the gradient
        """
        # TODO:
        # Compute and return the gradient
        # Your answer here
        return np.zeros_like(weights)

    def train(self, data, iterations=500, learningRate=0.01, weightDecay=0.001):
        """ Perform gradient descent training to compute weights

        Our optimization program should minimize the following:
            L(w) = lambda/2 * sum_{j=1,...,M} w_j^2 + sum_{i=1,...,N} 1/N [-(y_i log(p_i) + (1-y_i)log(1-p_i))]

        where:
            lambda = weightDecay, and the weight decay term does NOT apply to the bias entry (i.e., the very last weight).
            p_i is the prediction for sample i (i.e., f(x_i))
            y_i is 0 (not LEGO) or 1 (LEGO)

        The function should set the self.weights variable to the newly updated weights.
            
        :param data: data to use for evaluation. It is expected to be a dictionary, with the following
            - 'mug': list of raw images
            - 'lego': list of raw images
        :param iterations: iterations of gradient descent to run
        :param learningRate: learning rate to use (sometimes referred to as alpha)
        :param weightDecay: weight decay amount (sometimes referred to as lambda)
        :returns: loss for each iteration
        """
        # TODO:
        # Retrieve input data (Hint: use a function you implemented above)
        # Your answer here
        # X should be (N,C*H*W+1) and y should be (N,) where an entry of y being 0 means not LEGO for that x_i and 1 means it is LEGO
        X, y = np.zeros((len(data['mug']) + len(data['lego']), 3 * self._imageHeight * self._imageWidth + 1)), np.zeros(len(data['mug']) + len(data['lego']))

        losses = [] # loss for each iteration
        for iteration in range(iterations):
            # TODO:
            # Perform gradient descent by computing gradient and updating self.weights appropriately.
            # You should also compute the loss to help aid in monitoring the model's training.
            # The loss should be computed AFTER the gradient step.
            # When computing loss/gradient, clamp the value of predictions to [EPS, 1-EPS] (where EPS is a small constant defined at the top of the file)
            # Your answer here
            gradient = np.zeros_like(self.weights)
            self.weights = self.weights
            loss = 0.0
            # Collect loss
            losses.append(loss)
        
        return losses

    def eval(self, data, weights):
        """ Perform evaluation on the provided data by computing prediction accuracy

        You should use a label of 1 for LEGO and 0 for other (i.e., mug).
        In other words, you are doing a "LEGO" predictor, where the sigmoid output is the probability that the
        given image is LEGO (so 1.0 would be highly certain it's LEGO, and 0.0 would be highly certain its not LEGO).
        Use a decision threshold of 0.5 to separate the classes.
        
        :param data: data to use for evaluation. It is expected to be a dictionary, with the following
            - 'mug': list of raw images
            - 'lego': list of raw images
        :param weights: weights of model to evaluate
        :returns: mean binary accuracy measured over data (float in range 0.0 to 1.0)
        """
        # TODO:
        # Calculate the accuracy of the classifier
        # Your answer here
        return 0.0
    # -----------------------------------------------------------------------
