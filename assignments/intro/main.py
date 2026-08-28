#
# You do not need to modify this file.
#


import sys
sys.path.append('./')

import pybullet
import random
import numpy as np
from robot.simulation import Simulation
from robot.urdf_object import URDFObject
from robot.r2d2 import R2D2
from assignments.intro.implementation import Implementation
from assignments.intro.utils import plotOdometry3D


DELTA_TIME = 1.0 / 240.0
MAX_TIME_SECONDS = 60.0


def run(implementationClass, headless=False):
    """ Runs the main program """

    random.seed(123)
    np.random.seed(123)

    simulation = Simulation(DELTA_TIME, headless=headless, realtime=False if headless else True)

    floor = URDFObject("plane.urdf", fixedBase=True)
    robot = R2D2(position=(0.0, 0.0, 0.5), rotation=pybullet.getQuaternionFromEuler([0, 0, 0.0]) )
    robot.setJointMotor(8, 'position', -0.35) # Set arm to retracted

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
    plotOdometry3D(implementation, DELTA_TIME, 1.0)
