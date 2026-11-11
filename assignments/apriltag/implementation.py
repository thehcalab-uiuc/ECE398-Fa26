#
# This is the file you will be modifying.
#


import cv2
import pupil_apriltags
import numpy as np


CORNER_DIST = 0.4
CORNERS_3D = np.array([
    [-CORNER_DIST, -CORNER_DIST, 0.0], # Bottom-left
    [ CORNER_DIST, -CORNER_DIST, 0.0], # Bottom-right
    [ CORNER_DIST,  CORNER_DIST, 0.0], # Top-right
    [-CORNER_DIST,  CORNER_DIST, 0.0], # Top-left
])


class Implementation:
    def __init__(self, robot):
        self._robot = robot
        self._elapsed = 0.0
        self.done = False
        self._captureTimer = 0.0
        self._captureDeltaTime = 1.0 / 1.0 # 1 Hz
        self.cv2Solutions = []
        self.homographySolutions = []
        self.iterativeSolutions = []
    
    def controlLoop(self, deltaTime):
        """ Runs the control loop of the robot
        
        DO NOT MODIFY

        :param deltaTime: delta time between ticks
        """
        if self._elapsed < 1.0:
            self._robot.sendCommand(0.0, 0.0)
        elif self._elapsed < 5.0:
            self._robot.sendCommand(0.1, 0.0)
        elif self._elapsed < 10.0:
            self._robot.sendCommand(0.2, -0.1)
        elif self._elapsed < 15.0:
            self._robot.sendCommand(0.3, 0.0)
        elif self._elapsed < 20.0:
            self._robot.sendCommand(0.0, 0.1)
        elif self._elapsed < 25.0:
            self._robot.sendCommand(0.1, 0.0)
        elif self._elapsed < 30.0:
            self._robot.sendCommand(-0.2, 0.1)
        elif self._elapsed < 35.0:
            self._robot.sendCommand(-0.2, 0.0)
        elif self._elapsed < 40.0:
            self._robot.sendCommand(-0.1, -0.1)
        elif self._elapsed < 45.0:
            self._robot.sendCommand(-0.1, 0.0)
        else:
            self.done = True

    def aprilTagDetect(self, image):
        """ Detect AprilTags in an image
        
        DO NOT MODIFY

        :param image: OpenCV-formatted image (assumed BGR)
        :returns: list of detected markers, where each element is an np.array of corners (4,2) ordered bottom-left, bottom-right, top-right, top-left
        """
        detector = pupil_apriltags.Detector()
        results = detector.detect(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        return [np.array(result.corners) for result in results]
    
    def rvecToMatrix(self, rvec):
        """ Convert a rotation vector to a rotation matrix

        DO NOT MODIFY
        
        :param rvec: (3,) rotation vector
        :returns: (3,3) rotation matrix
        """
        return cv2.Rodrigues(rvec)[0]
    
    def matrixToRvec(self, R):
        """ Convert a rotation matrix to a rotation vector
        
        DO NOT MODIFY

        :param R: (3,3) rotation matrix
        :returns: (3,) rotation vector
        """
        return cv2.Rodrigues(R)[0][:,0]

    def solvePnP_cv2(self, intrinsicMatrix, imagePoints, objectPoints):
        """ Solves the PnP problem using OpenCV's implementation. Useful for double-checking your implementation.
        
        DO NOT MODIFY

        :param intrinsicMatrix: 3x3 camera intrinsic matrix
        :param imagePoints: points in the image (N,2)
        :param objectPoints: 3D locations of image points in their own coordinate system (N,3)
        :returns: tuple with translation (3,) and rotation (3,), or None if not successful
        """
        # Solve PnP
        success, rvec, tvec = cv2.solvePnP(objectPoints, imagePoints, intrinsicMatrix, np.zeros((4,1)))
        if success:
            return (tvec[:,0], rvec[:,0])
        else:
            return None
    
    def computeHomography_cv2(self, imagePoints, objectPoints):
        """ Compute homography matrix (3x3) for homography between image points and object points
        
        DO NOT MODIFY

        :param imagePoints: points in image (N,2)
        :param objectPoints: points in some 3D frame (N,3)
        :returns: 3x3 homography matrix
        """
        return cv2.findHomography(objectPoints, imagePoints)[0]
    
    def computeProjectionResiduals(self, intrinsicMatrix, imagePoints, objectPoints, transform):
        """ Compute the projection error

        DO NOT MODIFY

        :param intrinsicMatrix: 3x3 intrinsic matrix
        :param imagePoints: points in the image (N,2)
        :param objectPoints: 3D locations of image points in their own coordinate system (N,3)
        :param transform: predicted tuple of translation (3,) and rotation (3,) (i.e., tvec and rvec)
        :returns: projection residuals (N,2)
        """
        t, R = transform[0], self.rvecToMatrix(transform[1])
        fx, fy, cx, cy = intrinsicMatrix[0,0], intrinsicMatrix[1,1], intrinsicMatrix[0,2], intrinsicMatrix[1,2]
        points = ((R @ objectPoints.T).T + t[None,:])
        predImagePoints = np.stack([
            fx * points[:,0] / points[:,2] + cx,
            fy * points[:,1] / points[:,2] + cy
        ], axis=1)
        return imagePoints - predImagePoints
    
    # Your implementation!
    # -----------------------------------------------------------------------
    def skew(self, v):
        """ Compute skew for a vector

        :param v: 3 vector from which to compute the matrix
        :returns: 3x3 matrix that when multiplied acts like performing the cross product with the provided vector
        """
        # TODO:
        # Your answer here
        return np.eye(3)
        
    def initializationPnP(self, intrinsicMatrix, homography):
        """ Compute the analytic initialization point for LM PnP optimization
        
        :param intrinsicMatrix: 3x3 intrinsic matrix
        :param homography: 3x3 homography matrix
        :returns: tuple with translation (3,) and rotation (3,)
        """
        # TODO:
        # Find Hbar = s[r1,r2,t] for some scaling s.
        # Hint: this should be computed using intrinsicMatrix and homography, using the fact that H=sK[r1,r2,t]
        # Your answer here
        Hbar = np.zeros_like(intrinsicMatrix)
        
        # TODO:
        # Extract r1, r2, r3, and t from Hbar
        # Hint: make sure r1, r2, and r3 are unit vectors (use the mean of Hbar's first two columns to help get magnitude of scaling!), and that z is pointing towards us
        # Extra hint: use t to decide on the sign of the scaling factor (remember the object must be in front of us to have been detected!)
        # Your answer here
        r1 = np.array([1., 0., 0.])
        r2 = np.array([0., 1., 0.])
        r3 = np.array([0., 0., 1.])
        t = np.zeros((3,))
        
        # TODO:
        # Create a valid rotation matrix from [r1,r2,r3].
        # Hint: ensure it satisfies constraints of a rotation matrix (orthonormal, right-handed)
        # Hint: you can use SVD to ensure orthonormality.
        # Your answer here
        R = np.eye(3)
        
        # Return result
        return (t, self.matrixToRvec(R))
    
    def optimizePnP(self, intrinsicMatrix, imagePoints, objectPoints, transform, Linit=1.0, maxIterations=10000):
        """ Perform Levenberg Marquart optimization for PnP.

        For the implementation, each iteration should either update the solution (and decrease L) or skip the update and increase L.
        Use the following strategy for L (aka lambda):
            https://www.youtube.com/watch?v=UQsOyMj9lnI
            This rule involves updating the solution and multiplying L by 0.8 if error decreases, or not updating and increasing L by 2 times if error doesn't decrease.
        
        :param intrinsicMatrix: intrinic matrix for camera (3x3)
        :param imagePoints: points of tag corners in image (4,2)
        :param objectPoints: points of tag corners in some 3D frame (4,3)
        :param transform: initialization point as a tuple of translation (3,) and rotation (3,)
        :param Linit: initial lambda
        :param maxIterations: maximum number of iterations
        :returns: boolean success flag and tuple with translation (3,) and rotation (3,)
        """

        L = Linit
        fx, fy, cx, cy = intrinsicMatrix[0,0], intrinsicMatrix[1,1], intrinsicMatrix[0,2], intrinsicMatrix[1,2]
        solution = (transform[0].copy(), transform[1].copy())
        success = False

        for iter in range(maxIterations):
            if L > 1e10: # L has blown up
                break
            
            # Extract start point
            t, r = solution[0], solution[1]
            R = self.rvecToMatrix(r)

            # TODO:
            # Compute Jacobian
            # Your answer here
            J = np.zeros((8,6))

            # TODO:
            # Compute residuals and projection error
            # Hint: computeProjectionResiduals may be helpful
            # Your answer here:
            residuals = np.zeros((4,2)) # residuals to be used in projection error and computation of step (delta)
            projectionError = 0.0       # total projection error (sum of squares of residuals)

            # Convergence check
            if np.linalg.norm(J.T @ residuals.flatten()) < 1e-4:
                success = True
                break

            # TODO:
            # Compute the update and apply
            # Your answer here:
            delta = np.zeros((6,))
            tnew, rnew = t, r

            # TODO:
            # Update the solution if needed (i.e., if taking step leads to reduction in projection error) and adjust L (lambda).
            # Your answer here:
            projectionErrorNew = projectionError # TODO: calculate new projection error
            solution = (tnew.copy(), rnew.copy())
            L = L
        
        return (success, solution)

    def solvePnP(self, intrinsicMatrix, imagePoints, objectPoints, method='iterative'):
        """ Solve the PnP problem
        
        :param intrinsicMatrix: intrinic matrix for camera (3x3)
        :param imagePoints: points of tag corners in image (4,2)
        :param objectPoints: points of tag corners in some 3D frame (4,3)
        :param method: method to use ('homography' for purely analytic method, 'iterative' for Levenberg-Marquart where initialization given by analytic method)
        :returns: tuple with translation (3,) and rotation (3,), or None if not successful/converged
        """
        # Ensure method is supported
        if method not in ['homography', 'iterative']:
            raise ValueError("Invalid method")

        # First, we find the homography (use OpenCV)
        homography = self.computeHomography_cv2(imagePoints, objectPoints)

        # TODO:
        # Find the initialization point for our optimization by decomposing homography (hint: use self.initializationPnP)
        # Your answer here:
        tInit, rInit = np.zeros((3,)), np.zeros((3,))

        # Early-out if only using homography
        if method == 'homography':
            return (tInit, rInit)

        # TODO:
        # Perform Levenberg Marquart optimization (hint: use self.optimizePnP)
        # Your answer here:
        success, (tFinal, rFinal) = True, (np.zeros((3,)), np.zeros((3,)))

        if success:
            return (tFinal, rFinal)
        else:
            return None

    def getPoseInAprilTagCoordinates(self, tvec, rvec):
        """ Converts a pose (tvec, rvec) representing AprilTag relative to camera to a pose representing camera in AprilTag coordinate system
        
        :param tvec: translation vector for transformation representing AprilTag in camera frame
        :param rvec: rotation vector for transformation representing AprilTag in camera frame
        :returns: (tvec, rvec) representing transformation of camera in AprilTag frame
        """
        # TODO:
        # Find the (tvec, rvec) that correspond to the camera in the AprilTag coordinate system
        # Your answer here:
        return (tvec, rvec)
    # -----------------------------------------------------------------------

    def onTick(self, deltaTime):
        """ This function is called on each tick of the clock
        
        :param deltaTime: time between ticks
        """
        # Control the robot
        # -----------------------------------------------------------
        self.controlLoop(deltaTime)
        # -----------------------------------------------------------

        # Capture image, perform April Tag detection, and solve PnP
        # -----------------------------------------------------------
        if self._captureTimer > self._captureDeltaTime:
            intrinsicMatrix = self._robot.camera.getIntrinsicMatrix()
            image, _ = self._robot.captureImage()
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR) # OpenCV expects BGR image
            detections = self.aprilTagDetect(image)
            if len(detections) == 1:
                # Compute and store solutions
                pnpSolution_cv2 = self.solvePnP_cv2(intrinsicMatrix, detections[0], CORNERS_3D)
                if pnpSolution_cv2 is not None:
                    self.cv2Solutions.append({
                        'aprilTagInCamera': {'translation': pnpSolution_cv2[0], 'rotation': pnpSolution_cv2[1]},
                        'cameraInAprilTag': {k: v for k, v in zip(['translation', 'rotation'], self.getPoseInAprilTagCoordinates(*pnpSolution_cv2))},
                        'error': np.square(self.computeProjectionResiduals(intrinsicMatrix, detections[0], CORNERS_3D, pnpSolution_cv2)).sum()
                    })
                else:
                    self.cv2Solutions.append(None)
                    print("cv2 failed to find pnp solution")

                pnpSolution_analytic = self.solvePnP(intrinsicMatrix, detections[0], CORNERS_3D, method='homography')
                if pnpSolution_analytic is not None:
                    self.homographySolutions.append({
                        'aprilTagInCamera': {'translation': pnpSolution_analytic[0], 'rotation': pnpSolution_analytic[1]},
                        'cameraInAprilTag': {k: v for k, v in zip(['translation', 'rotation'], self.getPoseInAprilTagCoordinates(*pnpSolution_analytic))},
                        'error': np.square(self.computeProjectionResiduals(intrinsicMatrix, detections[0], CORNERS_3D, pnpSolution_analytic)).sum()
                    })
                else:
                    self.homographySolutions.append(None)
                    print("analytic method failed to find pnp solution")

                pnpSolution_LM = self.solvePnP(intrinsicMatrix, detections[0], CORNERS_3D, method='iterative')
                if pnpSolution_LM is not None:
                    self.iterativeSolutions.append({
                        'aprilTagInCamera': {'translation': pnpSolution_LM[0], 'rotation': pnpSolution_LM[1]},
                        'cameraInAprilTag': {k: v for k, v in zip(['translation', 'rotation'], self.getPoseInAprilTagCoordinates(*pnpSolution_LM))},
                        'error': np.square(self.computeProjectionResiduals(intrinsicMatrix, detections[0], CORNERS_3D, pnpSolution_LM)).sum()
                    })
                else:
                    self.iterativeSolutions.append(None)
                    print("LM method failed to find pnp solution")
            else:
                print('Skipping - wrong number of detections!')

            self._captureTimer = 0.0
        self._captureTimer += deltaTime
        # -----------------------------------------------------------

        self._elapsed += deltaTime
