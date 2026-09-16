#
# This is the file you will be modifying.
#


import random
import pybullet
import numpy as np
import scipy.spatial.transform


MAX_DEPTH_TOLERANCE_MULTIPLIER = 0.99
MAX_DEPTH = 5.0
TIME_TO_WAIT = 2.0 # seconds


class Implementation:
    def __init__(self, robot, planeVerticalOffset, planeRotation, useRANSAC):
        self._robot = robot
        self._elapsed = 0.0
        self.planeVerticalOffset = planeVerticalOffset
        self.planeRotation = planeRotation
        self.useRANSAC = useRANSAC
        self.groundtruth = None
        self.fitted = None
        self.done = False
        # Used for final output
        self.numPoints = 0
        self.numInliers = 0

    def getGroundtruthRobotPose(self):
        """ Get the groundtruth position of the robot (in world frame)
        
        DO NOT MODIFY

        :returns: tuple of 2 np.arrays (length 3 and a 3x3) corresponding to position and rotation matrix
        """
        robotWorldPosition, robotWorldRotation = self._robot.getBaseTransform()
        robotWorldRotationM = np.array(pybullet.getMatrixFromQuaternion(robotWorldRotation)).reshape(3, 3)
        return np.array(robotWorldPosition), robotWorldRotationM
    
    def controlLoop(self, deltaTime):
        """ Runs the control loop of the robot
        
        DO NOT MODIFY

        :param deltaTime: delta time between ticks
        """
        self._robot.sendCommand(0.0, 0.0)

    def projectTo3D(self, depth):
        """ Project a depth map to 3D points
        
        DO NOT MODIFY

        :param depth: depth image (H,W)
        :returns: projected points of shape (H,W,3)
        """
        H, W = depth.shape
        u, v = np.meshgrid(np.arange(W), np.arange(H)) # HxW

        K = self._robot.camera.getIntrinsicMatrix()
        fx, fy = K[0,0], K[1,1]
        cx, cy = K[0,2], K[1,2]
        
        xOptical = (u - cx) * depth / fx
        yOptical = (v - cy) * depth / fy
        zOptical = depth.copy()

        # Set any out of range to max depth
        zOptical[(zOptical >= (self._robot.camera.getMaxDepth()*MAX_DEPTH_TOLERANCE_MULTIPLIER)) | (zOptical >= MAX_DEPTH)] = np.nan
        # Set any negatives to nan
        zOptical[(zOptical < 0.0)] = np.nan

        return np.stack([xOptical, zOptical, -yOptical], axis=-1)
    
    def measureDepthImage(self):
        """ Measure a depth image
        
        DO NOT MODIFY

        :returns: scan cloud of shape (N,3) with N measurements and where each measurement is (x,y,z) in camera coordinate frame
        """
        image, depth = self._robot.captureImage()
        projectedDepth = self.projectTo3D(depth)[:,:,:].reshape(-1,3)
        projectedDepth = projectedDepth[~np.isnan(projectedDepth).any(axis=1)]
        return projectedDepth
    
    def computeGroundtruthGroundPlane(self, planeVerticalOffset, planeRotation):
        """ Compute the groundtruth plane equation

        DO NOT MODIFY

        Value returned will be array of [a,b,c,d] corresponding to plane of form:
            ax + by + cz + d = 0
        
        :param planeVerticalOffset: vertical offset of plane at x=0, y=0
        :param planeRotation: rotation of plane in world coordinates
        :returns: coefficients [a,b,c,d] as numpy array
        """
        normal = scipy.spatial.transform.Rotation.from_quat(planeRotation).apply(np.array([0.0, 0.0, 1.0])).tolist()
        a, b, c = normal[0], normal[1], normal[2]
        d = -np.array(normal) @ np.array([0,0,planeVerticalOffset])
        return np.array([a,b,c,d])
    
    def convertToCanonical(self, planeEquationCoefficients):
        """ Convert a plane equation's coefficients to a canonical form (as plane equation is up to scale)
        
        DO NOT MODIFY

        :param planeEquationCoefficients: coefficients [a,b,c,d] as numpy array
        :returns: plane coefficients (as numpy array) but where normal points z-up and is normalized
        """
        coeff = planeEquationCoefficients.copy()
        if np.dot(coeff[0:3], np.array([0.0, 0.0, 1.0])) < 0.0:
            coeff = -coeff
        scale = np.linalg.norm(coeff[0:3])
        return coeff / scale

    # Your implementation!
    # -----------------------------------------------------------------------
    def computePlaneEquation(self, points):
        """ Calculate the coefficients for the plane's equation
        
        You should use the normal equations (i.e., the equations that allow you to find the linear least squares solution).
        Value returned will be array of [a,b,c,d] corresponding to the parameters of a plane of form:
            ax + by + cz + d = 0

        Solving for the parameters of the plane requires using least squares. As discussed in class, this involves solving the normal equations.
        While you can solve by computing an inverse, a general rule of thumb when performing computations is that it is advisable to avoid inverses if possible.
        Hence, you should use np.linalg.solve to solve the normal equations.

        :param points: points to use for fitting the plane. Array with shape (N,3)
        :returns: coefficients of plane equation [a,b,c,d] as a numpy array
        """
        # TODO:
        # Your answer here
        return np.array([0.0, 0.0, 0.0, 0.0])
    
    def computePlaneEquationRANSAC(self, points, iterations=1000, outlierEpsilon=1e-3):
        """ Calculate the constants for the plane's equation using RANSAC
        
        You should use the normal equations (i.e., the equations that allow you to find the linear least squares solution), but incorporate RANSAC to improve robustness to outliers.

        Value returned will be a numpy array of [a,b,c,d] corresponding to plane of form:
            ax + by + cz + d = 0
        
        A point (x', y', z') will be considered an outlier for RANSAC it has the property:
            abs(ax' + by' + cz' + d) / sqrt(a*a + b*b + c*c) > outlierEpsilon
        
        Points that are not outliers are considered inliers.

        Use the variant of RANSAC where you simply take the "best" solution of all iterations; for simplicity, you do *not* need to refit on the consensus set.
        
        :param points: points to use for fitting the plane
        :param iterations: number of RANSAC iterations
        :param outlierEpsilon: threshold for outliers
        :returns: tuple containing coefficients of plane equation [a,b,c,d] (as a numpy array) and the number of inliers
        """
        # TODO:
        # Your answer here
        return (np.array([0.0, 0.0, 0.0, 0.0]), 0)

    def cameraPointsToWorld(self, cameraPosition, cameraRotation, points):
        """ Transform camera frame points to the world frame
        
        :param cameraPosition: position of camera in world (3-vector)
        :param cameraRotation: rotation matrix of camera in world (3x3 matrix)
        :param points: points in camera's coordinate frame as np array of shape (N,3)
        :returns: points but represented in the world coordinate frame (N,3)
        """
        # TODO:
        # Your answer here
        return np.zeros_like(points)

    def estimateGroundPlane(self, points, useRANSAC=False):
        """ Fit a ground plane from the points
        
        :param points: points in robot's camera frame (N, 3)
        :param useRANSAC: whether to use RANSAC
        :returns: the ground plane coefficients [a,b,c,d] as a numpy array
        """
        # Get the camera's world pose as a position and a rotation matrix
        cameraPosition, cameraRotation = self._robot.getCameraTransform()

        # Convert scan points to world frame
        pointsInWorld = self.cameraPointsToWorld(cameraPosition, cameraRotation, points)
        
        if useRANSAC:
            # Compute the plane equation using RANSAC
            # TODO:
            # Your answer here
            plane, numInliers = np.zeros(4), 0

            # Save metadata
            self.numInliers = numInliers
            self.numPoints = len(pointsInWorld)
        else:
            # Compute the plane equation without RANSAC
            # TODO:
            # Your answer here
            plane = np.zeros(4)

            # Save metadata
            self.numInliers = len(pointsInWorld) # Naively assume all are inliers if we don't use RANSAC
            self.numPoints = len(pointsInWorld)

        return plane
    # -----------------------------------------------------------------------

    def onTick(self, deltaTime):
        """ This function is called on each tick of the clock
        
        :param deltaTime: time between ticks
        """

        # Control the robot
        # -----------------------------------------------------------
        self.controlLoop(deltaTime)
        # -----------------------------------------------------------

        # Compute results and mark finished if applicable
        if self._elapsed > TIME_TO_WAIT:
            self.groundtruth = self.convertToCanonical(self.computeGroundtruthGroundPlane(self.planeVerticalOffset, self.planeRotation))
            self.fitted = self.convertToCanonical(self.estimateGroundPlane(self.measureDepthImage(), useRANSAC=self.useRANSAC))
            self.done = True

        self._elapsed += deltaTime
