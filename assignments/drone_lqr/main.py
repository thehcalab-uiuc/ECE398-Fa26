import sys
sys.path.append('./')

import pybullet
import argparse
import random
import numpy as np
from robot.simulation import Simulation
from robot.urdf_object import URDFObject
from robot.quadrotor import Quadrotor
from assignments.drone_lqr.implementation import Implementation


DELTA_TIME = 1.0 / 240.0
MAX_TIME_SECONDS = 30.0
MAX_FORCE = 5.0
MAX_TORQUE = 5.0
GRAVITY = 9.8


def run(implementationClass, headless=False, controlMethod='openloop', startRandom=False, applyPerturbations=False, seed=123, desiredPosition=(0.0, 0.0, 1.0)):
    """ Runs the main program """

    random.seed(seed)
    np.random.seed(seed)

    simulation = Simulation(DELTA_TIME, headless=headless, realtime=False if headless else True, gravity=GRAVITY)

    floor = URDFObject("plane.urdf", fixedBase=True)
    robot = Quadrotor(
        position=(random.uniform(-1.0, 1.0), random.uniform(-1.0, 1.0), random.uniform(0.5, 2.0)) if startRandom else (0.0, 0.0, 1.0),
        rotation=pybullet.getQuaternionFromEuler([random.uniform(-0.524, 0.524), random.uniform(-0.524, 0.524), random.uniform(-0.524, 0.524)] if startRandom else [0.0, 0.0, 0.0])
    )

    implementation = implementationClass(robot, controlMethod=controlMethod, gravity=GRAVITY, desiredPosition=desiredPosition)
    currentTime = 0.0
    for i in range(int(MAX_TIME_SECONDS / DELTA_TIME)):
        # Tick implementation
        implementation.onTick(deltaTime=DELTA_TIME)
        # Tick robot
        robot.tick(deltaTime=DELTA_TIME)
        simulation.step()
        # Apply perturbations if enabled
        if applyPerturbations and (currentTime < MAX_TIME_SECONDS*0.7) and random.random() < DELTA_TIME:
            if random.random() < 0.5:
                print("Applying random force perturbation")
                robot.applyForce(-1, [random.uniform(-MAX_FORCE, MAX_FORCE), random.uniform(-MAX_FORCE, MAX_FORCE), random.uniform(-MAX_FORCE, MAX_FORCE)], [0.0, 0.0, 0.0], 'link')
            else:
                print("Applying random torque perturbation!")
                robot.applyTorque(-1, [random.uniform(-MAX_TORQUE, MAX_TORQUE), random.uniform(-MAX_TORQUE, MAX_TORQUE), random.uniform(-MAX_TORQUE, MAX_TORQUE)], 'link')
        
        currentTime += DELTA_TIME

    simulation.cleanup()

    return implementation


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--controlMethod", type=str, default='openloop', choices=['openloop', 'lqr'])
    parser.add_argument("--startRandom", default=False, action='store_true')
    parser.add_argument("--applyPerturbations", default=False, action='store_true')
    parser.add_argument("--seed", type=int, default=123)
    args = parser.parse_args()

    implementation = run(Implementation, controlMethod=args.controlMethod, startRandom=args.startRandom, applyPerturbations=args.applyPerturbations, seed=args.seed)

    print("Desired final state: {}".format(implementation.lastDesiredState))
    print("Robot final state: {}".format(implementation.lastActualState))
    print("Magnitude of difference between final robot and desired states: {:.6f}".format(np.linalg.norm(implementation.lastActualState - implementation.lastDesiredState)))
