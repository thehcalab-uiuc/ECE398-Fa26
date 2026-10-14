#
# This is the file you will be modifying.
#


import pybullet
import numpy as np


class Implementation:
    def __init__(self, robot, positionNoiseStd=0.05, headingNoiseStd=0.09):
        """ Constructor for implementation
        
        DO NOT MODIFY
        """
        self._robot = robot
        self._elapsed = 0.0
        self._positionNoiseStd = positionNoiseStd
        self._headingNoiseStd = headingNoiseStd
        self._filterState = None
        self._filterCovariance = None
        self._processCovariance = np.diag([1e-5] * 3)
        self._measurementCovariance = np.diag([positionNoiseStd**2, positionNoiseStd**2, headingNoiseStd**2])
        self.done = False
        self.measurements = []
        self.kalmanFilterStates = []
        self.groundtruthOdometry = []

    def sendCommand(self, linear, angular):
        """ Send a command to the robot

        DO NOT MODIFY
        
        :param linear: linear velocity
        :param angular: angular velocity
        """
        self._robot.sendCommand(linear, angular)
    
    def getRobotMeasurement(self):
        """ Get the noisy position (2D) and heading of the robot (in world frame)

        DO NOT MODIFY

        The position measurements have standard deviation of `self._positionNoiseStd`
        The heading measurements have standard deviation of `self._headingNoiseStd`
        
        :returns: measurement as numpy array (3,)
        """
        robotWorldPosition, robotWorldRotation = self._robot.getBaseTransform()
        robotWorldRotationM = np.array(pybullet.getMatrixFromQuaternion(robotWorldRotation)).reshape(3, 3)
        robotYaw = float(np.arctan2(robotWorldRotationM[1,0], robotWorldRotationM[0,0]))
        return np.array(
            (robotWorldPosition[0:2] + np.random.normal([0.0, 0.0], [self._positionNoiseStd, self._positionNoiseStd])).tolist() + \
            (np.array([robotYaw]) + np.random.normal([0.0], [self._headingNoiseStd])).tolist()
        )

    def getGroundtruthRobotPose(self):
        """ Get the groundtruth position of the robot (in world frame)
        
        DO NOT MODIFY

        :returns: tuple of 2 np.arrays (length 3 and a 3x3) corresponding to position and rotation matrix
        """
        robotWorldPosition, robotWorldRotation = self._robot.getBaseTransform()
        robotWorldRotationM = np.array(pybullet.getMatrixFromQuaternion(robotWorldRotation)).reshape(3, 3)
        return np.array(robotWorldPosition), robotWorldRotationM
    
    def getControl(self, deltaTime):
        """ Control Loop

        DO NOT MODIFY

        :param deltaTime: delta time
        :returns: np.array (2,) with linear (x) velocity and angular (yaw) velocity, or None if finished
        """
        if self._elapsed < 5.0:
            return np.array([1.0/5.0, 0.0])
        elif self._elapsed < 10.0:
            return np.array([0.0, (np.pi/2)/5.0])
        elif self._elapsed < 15.0:
            return np.array([0.5/5, 0.0])
        elif self._elapsed < 20.0:
            return np.array([0.0, -(np.pi/2)/5.0])
        elif self._elapsed < 30.0:
            return np.array([1.0/5, 0.0])
        elif self._elapsed < 35.0:
            return np.array([0.0, -(np.pi/2)/5.0])
        elif self._elapsed < 40.0:
            return np.array([0.5/5, 0.0])
        elif self._elapsed < 45.0:
            return np.array([0.0, (np.pi/2)/5.0])
        elif self._elapsed < 50.0:
            return np.array([1.0/5.0, 0.0])
        elif self._elapsed < 55.0:
            return np.array([0.0, (np.pi/2)/5.0])
        elif self._elapsed < 65.0:
            return np.array([1.0/5.0, 0.0])
        elif self._elapsed < 70.0:
            return np.array([0.0, (np.pi/2)/5.0])
        elif self._elapsed < 75.0:
            return np.array([1.0/5.0, 0.0])
        elif self._elapsed < 80.0:
            return np.array([0.0, (np.pi/2)/5.0])
        elif self._elapsed < 85.0:
            return np.array([0.5/5, 0.0])
        elif self._elapsed < 90.0:
            return np.array([0.0, -(np.pi/2)/5.0])
        elif self._elapsed < 100.0:
            return np.array([1.0/5, 0.0])
        elif self._elapsed < 105.0:
            return np.array([0.0, -(np.pi/2)/5.0])
        elif self._elapsed < 110.0:
            return np.array([0.5/5, 0.0])
        elif self._elapsed < 115.0:
            return np.array([0.0, (np.pi/2)/5.0])
        elif self._elapsed < 120.0:
            return np.array([1.0/5.0, 0.0])
        elif self._elapsed < 125.0:
            return np.array([0.0, (np.pi/2)/5.0])
        elif self._elapsed < 135.0:
            return np.array([1.0/5.0, 0.0])
        else:
            return None

    # Your implementation
    # -----------------------------------------------------------------------
    def kalmanPredict(self, prevMean, prevCov, control, deltaTime):
        """ Apply the predict step of the (extended) Kalman Filter
        
        Note: the heading in mean (i.e., mean[2]) should always be between -pi and pi! Hint: you can easily enforce this with np.arctan2, np.sin, and np.cos.

        :param prevMean: last state mean as numpy array (3,)
        :param prevCov: last state covariance as numpy array (3,)
        :param control: control consisting of linear and angular velocity as numpy array (2,)
        :param deltaTime: delta time
        :returns: tuple containing new mean (3,) and covariance (3,3) as numpy arrays
        """
        # TODO: compute new mean and covariance.
        # Hint: self._measurementCovariance or self._processCovariance may be of use.
        return (prevMean, prevCov)

    def kalmanUpdate(self, prevMean, prevCov, measurement):
        """ Apply the update step of the (extended) Kalman Filter
        
        Note: the heading (e.g., mean[2]) should always be between -pi and pi!
        
        :param prevMean: last state mean as numpy array (3,)
        :param prevCov: last state covariance as numpy array (3,)
        :param measurement: measurement to use for update, as numpy array (3,)
        :returns: tuple containing new mean (3,) and covariance (3,3) as numpy arrays
        """
        # TODO: compute new mean and covariance.
        # Hint: self._measurementCovariance or self._processCovariance may be of use.
        return (prevMean, prevCov)
    
    def onTick(self, deltaTime):
        """ This function is called on each tick of the clock
        
        :param deltaTime: time between ticks
        """
        # Get measurement of pose
        measurement = self.getRobotMeasurement()

        # Initialize filter state with measurement
        if self._filterState is None:
            self._filterState = measurement.copy()
            self._filterCovariance = np.diag([1.0, 1.0, 1.0])
        else:
            # TODO: perform an operation with your filter here!
            # Your answer here
            self._filterState, self._filterCovariance = self._filterState, self._filterCovariance

        # Store the measurements for later visualization/checking
        self.measurements.append(measurement)
        self.kalmanFilterStates.append(self._filterState)
        self.groundtruthOdometry.append(self.getGroundtruthRobotPose())

        # Get controls and send to robot
        control = self.getControl(deltaTime) # (linear, angular)
        if control is None:
            self.done = True
            return
        self.sendCommand(control[0], control[1])

        # TODO: perform operation with your filter here!
        # Your answer here
        self._filterState, self._filterCovariance = self._filterState, self._filterCovariance
        
        # Update clock
        self._elapsed += deltaTime
    # -----------------------------------------------------------------------