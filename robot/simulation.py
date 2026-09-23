import time
import pybullet
import pybullet_data
from .urdf_object import URDFObject


class Simulation:
    def __init__(self, deltaTime, headless=False, realtime=True, gravity=9.8):
        self._deltaTime = deltaTime
        self._physicsClient = pybullet.connect(pybullet.DIRECT) if headless else pybullet.connect(pybullet.GUI)
        self._realtime = realtime
        self._gravity = gravity
        pybullet.setTimeStep(deltaTime)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath()) # optionally
        pybullet.setGravity(0, 0, -self._gravity)
    
    def reset(self):
        pybullet.resetSimulation()

    def step(self):
        pybullet.stepSimulation()
        if self._realtime:
            time.sleep(self._deltaTime)

    def performCD(self):
        pybullet.performCollisionDetection()

    def cleanup(self):
        pybullet.disconnect()
