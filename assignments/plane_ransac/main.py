import sys
sys.path.append('./')

import pybullet
import argparse
import random
import numpy as np
from robot.simulation import Simulation
from robot.urdf_object import URDFObject
from robot.r2d2 import R2D2
from assignments.plane_ransac.implementation import Implementation


DELTA_TIME = 1.0 / 240.0
MAX_TIME_SECONDS = 60.0


def run(implementationClass, headless=False, useRANSAC=False, numCubes=0):
    """ Runs the main program """

    random.seed(123)
    np.random.seed(123)

    simulation = Simulation(DELTA_TIME, headless=headless, realtime=False if headless else True)
    planeVerticalOffset = random.uniform(-1, 1)
    planeRotation = np.array(pybullet.getQuaternionFromEuler([random.uniform(-0.087, 0.087), random.uniform(-0.087, 0.087), 0.0])).tolist()
    floor = URDFObject("plane.urdf", position=[0, 0, planeVerticalOffset], rotation=planeRotation, fixedBase=True)

    cubes = []
    for i in range(numCubes):
        position = [random.uniform(-3, 3), random.uniform(0, 5), random.uniform(1.5, 2.5)]
        while (position[0]**2 + position[1]**2) < 1.0:
            position = [random.uniform(-3, 3), random.uniform(0, 5), random.uniform(1.5, 2.5)]
        rotation = pybullet.getQuaternionFromEuler([random.uniform(0.0, 6.283), random.uniform(0.0, 6.283) ,random.uniform(0.0, 6.283)])
        cubes.append(URDFObject("cube.urdf", position, rotation, globalScaling=random.uniform(0.1, 0.35)))

    robot = R2D2(position=(0.0, 0.0, 0.525 + planeVerticalOffset), rotation=planeRotation)
    robot.setJointMotor(8, 'position', -0.35) # Set arm to retracted

    implementation = implementationClass(robot, planeVerticalOffset, planeRotation, useRANSAC)
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
    parser = argparse.ArgumentParser()
    parser.add_argument("--numCubes", type=int, default=0)
    parser.add_argument("--useRANSAC", default=False, action='store_true')
    args = parser.parse_args()

    implementation = run(Implementation, numCubes=args.numCubes, useRANSAC=args.useRANSAC)

    print('\n')
    print('Groundtruth: {}'.format(implementation.groundtruth))
    print('Fitted: {}'.format(implementation.fitted))
    print('Angular absolute difference: {:.3f} deg, Height absolute difference: {:.3f} m'.format(
        abs((180.0/np.pi)*np.arccos(np.clip(np.dot(implementation.groundtruth[0:3], implementation.fitted[0:3]), -1.0, 1.0))),
        abs(implementation.groundtruth[3]-implementation.fitted[3])
    ))
    if args.useRANSAC:
        print("Number of inliers = {} / {} ({:.1f}%)".format(implementation.numInliers, implementation.numPoints, 100.0 * implementation.numInliers / implementation.numPoints))
    print('\n')
