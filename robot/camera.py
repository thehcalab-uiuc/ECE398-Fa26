import math
import pybullet
import numpy as np


class Camera:
    def __init__(
        self,
        width=640,
        height=480,
        fov=60.0,
        nearClip=0.01,
        farClip=20.0,
        position=None,
        rotation=None
    ):
        """ Camera
        
        :param width: image width
        :param height: image height
        :param fov: field of view
        :param nearClip: near clipping plane
        :param farClip: far clipping plane
        :param position: position to use (or None)
        :param rotation: rotation to use (or None)
        """
        self._width = width
        self._height = height
        self._fov = fov
        self._nearClip = nearClip
        self._farClip = farClip

        aspect = width / height
        self._projectionMatrix = pybullet.computeProjectionMatrixFOV(
            fov=fov,
            aspect=aspect,
            nearVal=nearClip,
            farVal=farClip
        )
        self._position = position
        self._rotation = rotation

    def getMaxDepth(self):
        """ Get maximum depth """
        return self._farClip

    def getIntrinsicMatrix(self):
        """ Get the intrinsic camera matrix (cv2-style) """
        fovRadians = self._fov * (math.pi / 180.0)
        fx = fy = self._height / (2 * math.tan(fovRadians / 2))
        cx = self._width / 2.0
        cy = self._height / 2.0
        return np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0,  0,  1]
        ])

    def render(self, position=None, rotation=None):
        """ Render an image and depth image from the provided (world) position and rotation
        
        :param position: position to render from
        :param rotation: rotation to render from
        :returns: (rgb, depth)
        """
        if position is None:
            position = self._position
        if rotation is None:
            rotation = self._rotation

        rotationM = np.array(pybullet.getMatrixFromQuaternion(rotation)).reshape(3,3)
        forward = rotationM @ np.array([0.0, 1.0, 0.0])
        up = rotationM @ np.array([0.0, 0.0, 1.0])

        viewMatrix = pybullet.computeViewMatrix(
            cameraEyePosition=position,
            cameraTargetPosition=np.array(position)+forward,
            cameraUpVector=up
        )
        _, _, rgba, depthBuffer, _ = pybullet.getCameraImage(
            width=self._width,
            height=self._height,
            viewMatrix=viewMatrix,
            projectionMatrix=self._projectionMatrix,
            renderer=pybullet.ER_BULLET_HARDWARE_OPENGL # could switch to slower pybullet.ER_TINY_RENDERER for PL
        )

        # Color image
        rgba = np.reshape(rgba, (self._height, self._width, 4))
        rgb = rgba[:, :, :3].astype(np.uint8)

        # Depth image
        depthBuffer = np.reshape(depthBuffer, (self._height, self._width))
        depth = self._farClip * self._nearClip / (self._farClip - (self._farClip - self._nearClip) * depthBuffer)

        return rgb, depth
