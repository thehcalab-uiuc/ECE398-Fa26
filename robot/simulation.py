import time
import pybullet
import pybullet_data


class Simulation:
    def __init__(self, deltaTime, headless=False, realtime=True):
        """ Simulation
        
        :param deltaTime: timestep
        :param headless: whether to run in headless mode (if False, use GUI)
        :param realtime: whether to run in real-time mode (if False, runs as fast as possible)
        """
        self._deltaTime = deltaTime
        self._physicsClient = pybullet.connect(pybullet.DIRECT) if headless else pybullet.connect(pybullet.GUI)
        self._realtime = realtime
        pybullet.setTimeStep(deltaTime)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())
        pybullet.setGravity(0, 0, -9.8)
    
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
