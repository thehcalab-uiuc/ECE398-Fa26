import numpy as np
from .urdf_object import URDFObject


class Quadrotor(URDFObject):
    def __init__(self, position=(0.0,0.0,0.0), rotation=(0.0,0.0,0.0,1.0)):
        super().__init__('robot/urdfs/drone/quadrotor.urdf', position=position, rotation=rotation, fixedBase=False)
        dynamicsInfo = self.getDynamicsInfo()
        self.mass = dynamicsInfo['mass']
        self.armLength = 0.175
        self.inertia = np.diag(dynamicsInfo['local_inertia_diag'])
    
    def tick(self, deltaTime):
        """ Tick """
        super().tick(deltaTime=deltaTime)

    def applyRotorCommands(self, forces):
        """ Apply rotor commands
        
        :param forces: forces to apply to each rotor. The order is f1, f2, f3, f4, where f1 is forward (+y), f2 is right (+x), f3 is backward (-y), and f4 is left (-x)
        """
        self.applyForce(-1, [0.0, 0.0, forces[0]], [ 0.0,  self.armLength, 0.0], 'link')
        self.applyForce(-1, [0.0, 0.0, forces[1]], [ self.armLength,  0.0, 0.0], 'link')
        self.applyForce(-1, [0.0, 0.0, forces[2]], [ 0.0, -self.armLength, 0.0], 'link')
        self.applyForce(-1, [0.0, 0.0, forces[3]], [-self.armLength,  0.0, 0.0], 'link')
