import sys
sys.path.append('./')

import cv2
import random
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
from assignments.pickplace.classifier import LogisticClassifier


def readImage(file):
    """ Read image file
    
    DO NOT MODIFY
    """
    return cv2.resize(cv2.imread(str(file)), (32,32))


def loadImages():
    """ Load images for training
    
    DO NOT MODIFY
    """
    return {
        'train': {
            'mug': [readImage(file) for file in sorted(list(Path('assignments/pickplace/images/train/mug').glob('*.jpg')))],
            'lego': [readImage(file) for file in sorted(list(Path('assignments/pickplace/images/train/lego').glob('*.jpg')))]
        },
        'test': {
            'mug': [readImage(file) for file in sorted(list(Path('assignments/pickplace/images/test/mug').glob('*.jpg')))],
            'lego': [readImage(file) for file in sorted(list(Path('assignments/pickplace/images/test/lego').glob('*.jpg')))] 
        }
    }


if __name__ == "__main__":
    # Set seed
    random.seed(123)
    np.random.seed(123)

    # Load image data
    data = loadImages()

    # Create classifier
    classifier = LogisticClassifier(32,32)

    # Show results for untrained classifier
    print('Training accuracy for untrained classifier: {:.1f}%'.format(100.0*classifier.eval(data['train'], classifier.weights)))
    print('Testing accuracy for untrained classifier: {:.1f}%'.format(100.0*classifier.eval(data['test'], classifier.weights)))

    # Train
    losses = classifier.train(data['train'])

    # Show results for trained classifier
    print('Training accuracy for trained classifier: {:.1f}%'.format(100.0*classifier.eval(data['train'], classifier.weights)))
    print('Testing accuracy for trained classifier: {:.1f}%'.format(100.0*classifier.eval(data['test'], classifier.weights)))

    # Plot the loss over iterations
    plt.figure(figsize=(6,4))
    plt.plot(range(1, len(losses)+1), losses)
    plt.xlabel('iteration')
    plt.ylabel('loss')
    plt.show()
