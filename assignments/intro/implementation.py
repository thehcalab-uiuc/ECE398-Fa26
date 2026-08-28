#
# This is the file you will be modifying.
#


import math
import pybullet
import numpy as np


EPS = 1e-7


class Implementation:
    def __init__(self, robot):
        self._robot = robot
        self._elapsed = 0.0
        self._stateTime = 5.0 # Time to stay in a given state/segment
        self._segmentDistance = 2.5 # Distance to move for each segment
        self._segmentRotation = math.pi/2 # Radians to rotate for each segment
        self.done = False
        self.odometry = [self.getGroundtruthRobotPose()] # Stores the measured dead-reckoning odometry poses
        self.groundtruthOdometry = [self.getGroundtruthRobotPose()] # Stores the groundtruth odometry poses

    def sendCommand(self, linear, angular):
        """ Send a command to the robot

        DO NOT MODIFY
        
        :param linear: linear velocity
        :param angular: angular velocity
        """
        self._robot.sendCommand(linear, angular)

    def getRobotVelocity(self):
        """ Get the noisy velocity of the robot (in robot frame)
        
        DO NOT MODIFY

        :returns: tuple of 2 np arrays (each of length 3) corresponding to linear and angular velocity (noisy)
        """
        linearVelocity, angularVelocity = self._robot.getBaseLocalVelocity()
        linearVelocity = np.array(linearVelocity) + np.random.normal(loc=0.0, scale=0.3, size=(3,))
        angularVelocity = np.array(angularVelocity) + np.random.normal(loc=0.0, scale=0.6, size=(3,))
        return linearVelocity, angularVelocity
    
    def getGroundtruthRobotPose(self):
        """ Get the groundtruth position of the robot (in world frame)
        
        DO NOT MODIFY

        :returns: tuple of 2 np.arrays (length 3 and a 3x3) corresponding to position and rotation matrix
        """
        robotWorldPosition, robotWorldRotation = self._robot.getBaseTransform()
        robotWorldRotationM = np.array(pybullet.getMatrixFromQuaternion(robotWorldRotation)).reshape(3, 3)
        return np.array(robotWorldPosition), robotWorldRotationM

    # Part 1 - Control the robot
    # -----------------------------------------------------------------------
    def controlLoop(self):
        """ Control Loop for Part 1
        
        The goal is for the robot to move in a specific pattern (a square motion with each edge being of length self._segmentDistance)

        All segments last self._stateTime time units and assume constant controls.
        - For the first segment (lasting self._stateTime and starting at t=0), it should use a constant open-loop command that would cover self._segmentDistance linearly and not rotate.
        - For the second segment (lasting self._stateTime at starting at t=self._stateTime), it should use a constant open-loop command where it does not move linearly but should rotate by self._segmentRotation radians.
        - For the third segment, fifth segment, and seventh segment, do the same as for the first segment.
        - For the fourth and sixth segment, do the same as for the second segment.
        - On the 8th segment, the robot should stop.
        """
        if self._elapsed < 1*self._stateTime:
            # TODO:
            # Your answer here
            pass
        elif self._elapsed < 2*self._stateTime:
            # TODO:
            # Your answer here
            pass
        elif self._elapsed < 3*self._stateTime:
            # TODO:
            # Your answer here
            pass
        elif self._elapsed < 4*self._stateTime:
            # TODO:
            # Your answer here
            pass
        elif self._elapsed < 5*self._stateTime:
            # TODO:
            # Your answer here
            pass
        elif self._elapsed < 6*self._stateTime:
            # TODO:
            # Your answer here
            pass
        elif self._elapsed < 7*self._stateTime:
            # TODO:
            # Your answer here
            pass
        else:
            pass
            self.done = True
    # -----------------------------------------------------------------------

    # Part 2 - Measure odometry of robot by integrating velocity measurements
    # -----------------------------------------------------------------------
    def skew(self, v):
        """ Compute skew for a vector

        :param v: 3 vector from which to compute the matrix
        :returns: 3x3 matrix corresponding that when multiplied acts like performing the cross product with the provided vector
        """
        # TODO:
        # Your answer here
        return np.zeros((3,3))
    
    def rodrigues(self, omega, deltaTime):
        """ Compute rotation matrix from omega and deltaTime
        
        Note:
            The omega vector does not have to be a unit vector for this function. Instead, assume that you rotate by a unit vector in the direction of omega but by an angle specified by norm of omega times deltaTime.
            To detect the edge case in which the case that omega is zero vector, use a threshold of EPS (i.e., assume omega is zero if its norm is below EPS).

        :param omega: rotation (angle-axis) as 3 vector
        :param deltaTime: delta time to rotate by
        :returns: 3x3 rotation matrix
        """
        # TODO:
        # Your answer here
        return np.zeros((3,3))

    def measureOdometry(self, deltaTime):
        """ Odometry dead reckoning for Part 2
        
        The goal is to integrate linear/angular velocities over time (first-order Euler update)
        
        :param deltaTime: tick delta time
        :returns: two np arrays, one is a 3 vector of the new position and the other is a 3x3 matrix with the new rotation
        """
        robotVelocity, robotAngularVelocity = self.getRobotVelocity()
        priorOdomPosition, priorOdomRotation = self.odometry[-1]
        
        # TODO:
        # Your answer here
        return np.zeros_like(priorOdomPosition), np.zeros_like(priorOdomRotation)
    # -----------------------------------------------------------------------
    
    def onTick(self, deltaTime):
        """ This function is called on each tick of the clock
        
        :param deltaTime: time between ticks
        """
        # Part 1 - Control the robot
        # -----------------------------------------------------------
        self.controlLoop()
        # -----------------------------------------------------------

        # Part 2 - Measure odometry
        # -----------------------------------------------------------
        self.odometry.append(self.measureOdometry(deltaTime))
        self.groundtruthOdometry.append(self.getGroundtruthRobotPose())
        # -----------------------------------------------------------
        
        self._elapsed += deltaTime
