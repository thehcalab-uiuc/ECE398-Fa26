import sys
sys.path.append('./')

import pybullet
import random
import numpy as np
import matplotlib.pyplot as plt
from robot.simulation import Simulation
from robot.urdf_object import URDFObject
from robot.r2d2 import R2D2
from assignments.occupancy.implementation import Implementation


DELTA_TIME = 1.0 / 240.0
MAX_TIME_SECONDS = 60.0


def run(implementationClass, headless=False):
    """ Runs the main program """

    random.seed(123)
    np.random.seed(123)
    
    simulation = Simulation(DELTA_TIME, headless=headless, realtime=False if headless else True)

    # Environment
    floor = URDFObject("plane.urdf", fixedBase=True)
    cube1 = URDFObject("cube.urdf", position=[-2,2,0.5], rotation=pybullet.getQuaternionFromEuler([0, 0, 0]), fixedBase=True, globalScaling=1.5)
    cube1.setBaseColor((1,0,0,1))
    cube2 = URDFObject("cube.urdf", position=[-1,5,0.5], rotation=pybullet.getQuaternionFromEuler([0, 0, 3.14/4]), fixedBase=True, globalScaling=1.5)
    cube2.setBaseColor((0,1,0,1))
    cube3 = URDFObject("cube.urdf", position=[5,3,0.5], rotation=pybullet.getQuaternionFromEuler([0, 0, 3.14/4]), fixedBase=True, globalScaling=1.5)
    cube3.setBaseColor((0,0,1,1))
    cube4 = URDFObject("cube.urdf", position=[2,7,0.5], rotation=pybullet.getQuaternionFromEuler([0, 0, 0]), fixedBase=True, globalScaling=1.5)
    cube4.setBaseColor((0,0,1,1))
    cube5 = URDFObject("cube.urdf", position=[2,-4,0.5], rotation=pybullet.getQuaternionFromEuler([0, 0, 0]), fixedBase=True, globalScaling=1.5)
    cube5.setBaseColor((0,0,1,1))
    cube6 = URDFObject("cube.urdf", position=[-2,-3,0.5], rotation=pybullet.getQuaternionFromEuler([0, 0, 3.14/4]), fixedBase=True, globalScaling=1.5)
    cube6.setBaseColor((1,1,0,1))
    if headless == False:
        samurai = URDFObject("samurai.urdf", position=[-2, 0, 0], rotation=pybullet.getQuaternionFromEuler([0, 0, 0.0]), fixedBase=True)

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
    plt.figure()
    plt.imshow(implementation.probabilityMap, vmin=0.0, vmax=1.0, cmap='grey')
    plt.show()
