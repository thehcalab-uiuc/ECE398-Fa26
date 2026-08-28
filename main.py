#
# Simple test
#

import pybullet
from robot.simulation import Simulation
from robot.urdf_object import URDFObject
from robot.r2d2 import R2D2


DELTA_TIME = 1.0 / 240.0
MAX_TIME_SECONDS = 5*60.0


if __name__ == "__main__":
    simulation = Simulation(DELTA_TIME)
    floor = URDFObject("plane.urdf", fixedBase=True)
    robot = R2D2(position=(0.0, 0.0, 1.0), rotation=pybullet.getQuaternionFromEuler([0, 0, 1.0*3.14159/2]) )
    
    for i in range(int(MAX_TIME_SECONDS / DELTA_TIME)):
        robot.tick(DELTA_TIME)
        simulation.step()

    simulation.cleanup()
