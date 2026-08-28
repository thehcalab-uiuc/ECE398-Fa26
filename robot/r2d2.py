import pybullet
import numpy as np
from .camera import Camera
from .urdf_object import URDFObject
from .differential_drive import DifferentialDrive


MAX_LINEAR_SPEED = 1.0
MAX_ANGULAR_SPEED = 1.0


class R2D2(URDFObject):
    def __init__(self, position=(0.0,0.0,0.0), rotation=(0.0,0.0,0.0,1.0)):
        super().__init__('r2d2.urdf', position=position, rotation=rotation, fixedBase=False)
        self.controller = DifferentialDrive(
            self,
            leftWheelJoints=['left_front_wheel_joint', 'left_back_wheel_joint'],
            rightWheelJoints=['right_front_wheel_joint', 'right_back_wheel_joint'],
            wheelSeparation=0.44,
            wheelRadius=0.035,
            wheelSeparationMultiplier=1.0,
            wheelRadiusMultiplier=1.0
        )
        self.camera = Camera(
            width=640, height=480, fov=60.0, nearClip=0.01, farClip=10.0
        )

    def tick(self, deltaTime):
        """ Tick """
        super().tick(deltaTime=deltaTime)
        self.controller.tick(deltaTime=deltaTime)

    def sendCommand(self, linear, angular):
        """ Send a command to R2D2 """
        self.controller.sendCommand(
            np.clip(linear, min=-MAX_LINEAR_SPEED, max=MAX_LINEAR_SPEED),
            np.clip(angular, min=-MAX_ANGULAR_SPEED, max=MAX_ANGULAR_SPEED)
        )

    def captureImage(self):
        """ Capture an image from R2D2's camera """
        cameraPosition, cameraRotation = self.getCameraTransform(rotationAsQuat=True)
        return self.camera.render(cameraPosition, cameraRotation)
    
    def getCameraTransform(self, rotationAsQuat=False):
        """ Provide the camera's pose """
        headLinkState = self.getLinkState(14)
        cameraPosition, cameraRotation = headLinkState['worldPosition'], headLinkState['worldRotation']
        cameraRotationM = np.array(pybullet.getMatrixFromQuaternion(cameraRotation)).reshape(3,3)
        cameraPositionOffset = np.array(cameraPosition) + cameraRotationM @ np.array([0.0, 0.0, 0.0])
        if rotationAsQuat:
            return np.array(cameraPositionOffset), np.array(cameraRotation)
        else:
            return np.array(cameraPosition), np.array(cameraRotationM)
