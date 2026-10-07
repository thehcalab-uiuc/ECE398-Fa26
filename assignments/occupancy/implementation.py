#
# This is the file you will be modifying.
#


import skimage
import pybullet
import numpy as np


MAX_DEPTH_TOLERANCE_MULTIPLIER = 0.99


class Implementation:
    def __init__(self, robot):
        """ DO NOT MODIFY """
        self._robot = robot
        self._elapsed = 0.0
        self._captureTimer = 0.0
        self._captureDeltaTime = 1.0 / 5.0 # 5 Hz
        self.done = False

        # Probabilities for occupied, prior, and free
        self._pOcc = 0.9
        self._pPrior = 0.5
        self._pFree = 0.1

        self._mapSizeMetric = (15.0, 15.0) # m
        self._mapResolution = 0.1 # m / pixel
        self._mapSizeCells = (int(self._mapSizeMetric[0] / self._mapResolution), int(self._mapSizeMetric[1] / self._mapResolution)) # map resolution in cells

        # Stored occupancy map in log-odds form
        self.logOddsMap = self.probabilityToLogOdds(self._pPrior) * np.ones(self._mapSizeCells)
        self.probabilityMap = self.logOddsToProbability(self.logOddsMap)

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
        if self._elapsed < 5.0:
            self._robot.sendCommand(0.6, -0.3)
        elif self._elapsed < 10.0:
            self._robot.sendCommand(0.8, 0.4)
        elif self._elapsed < 15.0:
            self._robot.sendCommand(0.2, 0.5)
        elif self._elapsed < 20.0:
            self._robot.sendCommand(0.4, 0.1)
        elif self._elapsed < 25.0:
            self._robot.sendCommand(0.2, -0.3)
        else:
            self.done = True

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
        zOptical[(zOptical >= (self._robot.camera.getMaxDepth()*MAX_DEPTH_TOLERANCE_MULTIPLIER))] = self._robot.camera.getMaxDepth()
        # Set any negatives to nan
        zOptical[(zOptical < 0.0)] = np.nan

        return np.stack([xOptical, zOptical, -yOptical], axis=-1)
    
    def measureDepthScan(self):
        """ Measure a depth image scan
        
        DO NOT MODIFY

        :returns: scan line of shape (N,3) with N measurements (left->right) and where each measurement is (x,y,z) in camera coordinate frame
        """
        image, depth = self._robot.captureImage()
        projectedDepth = self.projectTo3D(depth)[depth.shape[0]//2,:,:] # central horizontal
        return projectedDepth[~np.isnan(projectedDepth).any(axis=1)] # remove NaN
    
    def pointsToGridmapLocations(self, pts):
        """ Convert array of 3D points to grid map cell locations

        DO NOT MODIFY
        
        :param pts: np array of shape (N,3)
        :returns: np array of shape (N,2) corresponding to grid map cell locations (may be out of bounds!)
        """
        return np.stack((
            pts[:,0] / self._mapResolution + self._mapSizeCells[0]//2,
            pts[:,1] / self._mapResolution + self._mapSizeCells[1]//2
        ), axis=-1).astype(np.int32)
    
    def restrictToCellsInMap(self, cells):
        """ Select the cells that are within the gridmap bounds

        DO NOT MODIFY
        
        :param cells: cells to select from
        :returns: cells, but where we removed entries that correspond to invalid indices (out of the grid)
        """
        return cells[(cells[:,0] >= 0) & (cells[:,0] < self._mapSizeCells[0]) & (cells[:,1] >= 0) & (cells[:,1] < self._mapSizeCells[1])]
    
    # Your implementation!
    # -----------------------------------------------------------------------
    def probabilityToLogOdds(self, p):
        """ Convert probability to log odds """
        # TODO:
        # Your answer here
        return np.zeros_like(p)
    
    def logOddsToProbability(self, l):
        """ Convert log odds to a probability """
        # TODO:
        # Your answer here
        return np.zeros_like(l)

    def scanPointsToWorld(self, cameraPosition, cameraRotation, scanPoints):
        """ Convert scan points from camera frame to the world frame

        :param cameraPosition: camera position as numpy array
        :param cameraRotation: camera rotation matrix as numpy array
        :param scanPoints: scan points as numpy array (N,3)
        :returns: scan points but converted into world frame as numpy array (N,3)
        """
        # TODO:
        # Your answer here
        return np.zeros_like(scanPoints)

    def updateOccupancyMap(self, cameraPosition, cameraRotation, logOddsMap, scanPoints):
        """ Update the occupancy map

        :param cameraPosition: camera position as numpy array
        :param cameraRotation: camera rotation matrix as numpy array
        :param logOddsMap: prior log odds map (as of the prior measurement)
        :param scanPoints: (N,3) array of points, where each of the N points has (x,y,z) coordinates in camera frame
        :returns: the probability occupancy map and the log odds map
        """
        # Any point where this mask is False corresponds to a point that did not have a return, so it returned infinite
        # depth but that is clamped to a finite depth of self._robot.camera.getMaxDepth(). Ignore these for filling markedCells,
        # but don't ignore when tracing for free cells.
        finitePointMask = scanPoints[:,1] < (self._robot.camera.getMaxDepth() - 1e-6)

        # Fill out the sections corresponding to the TODO's

        # Convert scan points to world frame
        scanPointsWorld = np.zeros_like(scanPoints) # TODO: calculate the scan points in world frame. Hint: use a function you implemented.

        # Convert scan points and camera position to to gridmap frame
        scanPointsGrid = self.pointsToGridmapLocations(scanPointsWorld)
        cameraPointGrid = self.pointsToGridmapLocations(cameraPosition[None,:])[0]

        # Collect the free cells
        freeCells = np.zeros((0,2), dtype=np.int32) # TODO: calculate which cell indices are free via line-tracing. Hint: use skimage.draw.line

        # Collect the marked cells (cells that the rays hit)
        markedCells = np.zeros((0,2), dtype=np.int32) # TODO: figure out which cells are occupied. Hint: marked cells are cells corresponding to (finite) hit returns (do we have a variable defined previously that holds this info?)

        # Restrict the free and marked cells to ensure indexing remains valid
        freeCells = self.restrictToCellsInMap(freeCells)
        markedCells = self.restrictToCellsInMap(markedCells)

        # Calculate log odds for inverse sensor model
        logOddsInvSensorModel = np.zeros_like(logOddsMap)
        logOddsInvSensorModel[freeCells[:,0], freeCells[:,1]] = 0.0 # TODO: set to the correct log-odds.
        logOddsInvSensorModel[markedCells[:,0], markedCells[:,1]] = 0.0 # TODO: set to the correct log-odds.

        # Calculate log odds for prior
        logOddsPrior = np.ones_like(logOddsMap) # TODO: set to the correct log-odds.

        # Compute updated log odds map
        updatedLogOddsMap = np.zeros_like(logOddsMap) # TODO: calculate the log odds map. Hint: use logOddsMap, logOddsInvSensorModel, and logOddsPrior.

        # Convert updated log odds map to occupancy probability map
        updatedOccupancyMap = np.zeros_like(logOddsMap) # TODO: calculate the final probability map.
        return updatedOccupancyMap, updatedLogOddsMap
    # -----------------------------------------------------------------------

    def onTick(self, deltaTime):
        """ This function is called on each tick of the clock

        DO NOT MODIFY
        
        :param deltaTime: time between ticks
        """
        # Control the robot
        # -----------------------------------------------------------
        self.controlLoop(deltaTime)
        # -----------------------------------------------------------

        # Capture depth image
        # -----------------------------------------------------------
        if self._captureTimer > self._captureDeltaTime:
            self.probabilityMap, self.logOddsMap = self.updateOccupancyMap(*self._robot.getCameraTransform(), self.logOddsMap, self.measureDepthScan())
            self._captureTimer = 0.0
        self._captureTimer += deltaTime
        # -----------------------------------------------------------

        # Mark finished if applicable
        if self._elapsed > 60.0:
            self.done = True

        self._elapsed += deltaTime
