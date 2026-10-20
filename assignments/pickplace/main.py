import sys
sys.path.append('./')

import pybullet
import random
import numpy as np
from robot.simulation import Simulation
from robot.urdf_object import URDFObject
from robot.kuka import Kuka
from robot.camera import Camera
from assignments.pickplace.implementation import Implementation
from assignments.pickplace.train import loadImages
from assignments.pickplace.classifier import LogisticClassifier


DELTA_TIME = 1.0 / 240.0
MAX_TIME_SECONDS = 60.0


def run(implementationClass, headless=False, seed=123):
    """ Runs the main program """

    if seed is not None:
        random.seed(seed)
        np.random.seed(seed)
    simulation = Simulation(DELTA_TIME, headless=headless, realtime=False if headless else True)
        
    # Create floors and tables
    floor = URDFObject("plane.urdf", fixedBase=True)
    table = URDFObject("table/table.urdf", fixedBase=True, position=[0.0, 0.65, 0.0])
    tableSquare = URDFObject("table_square/table_square.urdf", fixedBase=True, position=[0.0, -0.15, 0.0])

    # Set up robot and its camera
    robot = Kuka(position=(0.0, 0.0, 0.6), rotation=pybullet.getQuaternionFromEuler([0, 0, -np.pi/2.0]))
    camera = Camera(64, 64, fov=20.0, position=[0.9, 0.0, 1.25], rotation=pybullet.getQuaternionFromEuler([-np.pi/4, 0.0, np.pi/4]))

    tray1 = URDFObject("tray/tray.urdf", fixedBase=False, position=[-0.2, 0.4,  0.65], globalScaling=0.5)
    tray2 = URDFObject("tray/tray.urdf", fixedBase=False, position=[-0.2, 0.75, 0.65], globalScaling=0.5)

    # Object to grasp
    if random.random() < 0.5:
        object = URDFObject("urdf/mug.urdf", fixedBase=False, position=[0.5, 0.4, 0.7], rotation=pybullet.getQuaternionFromEuler([0.0, 0.0, random.uniform(0.0,2*np.pi)]), globalScaling=1.0)
        trueObjectLabel = 'mug'
    else:
        object = URDFObject("lego/lego.urdf", fixedBase=False, position=[0.5, 0.4, 0.7], rotation=pybullet.getQuaternionFromEuler([0.0, 0.0, random.uniform(0.0,2*np.pi)]), globalScaling=3.0)
        trueObjectLabel = 'lego'

    # Create and train classifier
    data = loadImages()
    classifier = LogisticClassifier(32,32)
    losses = classifier.train(data['train'])
    trainAccuracy = classifier.eval(data['train'], classifier.weights)
    evalAccuracy = classifier.eval(data['test'], classifier.weights)
    print('Training accuracy for trained classifier: {:.1f}%'.format(100.0 * trainAccuracy))
    print('Testing accuracy for trained classifier: {:.1f}%'.format(100.0 * evalAccuracy))

    # Create implementation and run
    implementation = implementationClass(robot, camera, object, classifier, trueObjectLabel)
    currentTime = 0.0
    for i in range(int(MAX_TIME_SECONDS / DELTA_TIME)):
        if currentTime > 1.0:
            # Main functionality after a second to prevent issues with initialization
            implementation.onTick(deltaTime=DELTA_TIME)
            
        robot.tick(DELTA_TIME)
        simulation.step()
        currentTime += DELTA_TIME

        if implementation.done:
            break
    
    simulation.cleanup()
    
    return implementation


if __name__ == "__main__":
    implementation = run(Implementation, seed=None)
