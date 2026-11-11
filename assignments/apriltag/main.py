import sys
sys.path.append('./')

import pybullet
import random
import numpy as np
from robot.simulation import Simulation
from robot.urdf_object import URDFObject
from robot.r2d2 import R2D2
from assignments.apriltag.utils import createAprilBox, summarize
from assignments.apriltag.implementation import Implementation


DELTA_TIME = 1.0 / 240.0
MAX_TIME_SECONDS = 120.0


def run(implementationClass, headless=False):
    """ Runs the main program """

    random.seed(123)
    np.random.seed(123)
    simulation = Simulation(DELTA_TIME, headless=headless, realtime=False if headless else True)
    floor = URDFObject("plane.urdf", position=[0,0,0], fixedBase=True)
    robot = R2D2(position=(0.0, 0.0, 0.5), rotation=pybullet.getQuaternionFromEuler([0.0, 0.0, 0.0]))
    robot.setJointMotor(8, 'position', -0.35) # Set arm to retracted

    createAprilBox([1.0, 5.5, 0.5], scale=1.0)

    implementation = implementationClass(robot)
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
    implementation = run(Implementation)
    summarize(implementation)
