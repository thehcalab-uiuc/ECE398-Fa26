import pybullet
import numpy as np
from .urdf_object import URDFObject

#
# Based on:
# https://github.com/bulletphysics/bullet3/blob/master/examples/pybullet/examples/inverse_kinematics.py
#

class Kuka(URDFObject):
    def __init__(self, position=(0.0,0.0,0.0), rotation=(0.0,0.0,0.0,1.0), useNullSpace=True):
        super().__init__('kuka_iiwa/model.urdf', position=position, rotation=rotation, fixedBase=True)
        self._endEffectorIndex = 6
        self._numJoints = pybullet.getNumJoints(self._objectID)
        if self._numJoints != 7:
            raise ValueError("Invalid number of joints")
        
        self._useNullSpace = useNullSpace
        self._ikSolver = 0
        self._lowerLimits  = [-0.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
        self._upperLimits  = [ 0.967,  2,  2.96, 2.29,  2.96,  2.09,  3.05]
        self._jointRanges  = [5.8, 4.0, 5.8, 4.0, 5.8, 4.0, 6]
        self._restPoses    = [0.0, 0.0, 0.0, 0.5 * np.pi, 0, -np.pi * 0.5 * 0.66, 0.0]
        self._jointDamping = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

        self._desiredPosition = (position[0] + 0.0, position[1] + 0.3, position[2] + 0.5)
        self._desiredRotation = (0.0, -1.0, 0.0, 0.0)

        # Reset
        for i in range(self._numJoints):
            pybullet.resetJointState(self._objectID, i, self._restPoses[i])

    def _performIK(self):
        """ Perform inverse kinematics and motor control """
        if self._useNullSpace:
            jointPoses = pybullet.calculateInverseKinematics(
                self._objectID,
                self._endEffectorIndex,
                self._desiredPosition,
                self._desiredRotation,
                self._lowerLimits,
                self._upperLimits,
                self._jointRanges,
                self._restPoses
            )
        else:
            jointPoses = pybullet.calculateInverseKinematics(
                self._objectID,
                self._endEffectorIndex,
                self._desiredPosition,
                self._desiredRotation,
                jointDamping=self._jointDamping,
                solver=self._ikSolver,
                maxNumIterations=100,
                residualThreshold=0.01
            )
        
        for i in range(self._numJoints):
            pybullet.setJointMotorControl2(
                bodyIndex=self._objectID,
                jointIndex=i,
                controlMode=pybullet.POSITION_CONTROL,
                targetPosition=jointPoses[i],
                targetVelocity=0,
                force=500,
                positionGain=0.03,
                velocityGain=1
            )

    def _getEndEffectorPose(self):
        """ Get the position and orientation of end effector in world-frame
        
        :returns: position (3,) and orientation (4,)
        """
        return pybullet.getLinkState(self._objectID, self._endEffectorIndex)[4:6]

    def sendEndEffectorPose(self, position, rotation=(0.0, -1.0, 0.0, 0.0)):
        """ Send the desired position and rotation to be achieved by the end effector """
        self._desiredPosition = position
        self._desiredRotation = rotation

    def distanceFromGoal(self):
        """ Returns the distance of the end effector from its desired position """
        return np.linalg.norm(np.array(self._desiredPosition) - np.array(self._getEndEffectorPose()[0]))

    def tick(self, deltaTime):
        """ Tick """
        super().tick(deltaTime=deltaTime)
        self._performIK()

    def addConstraintToEndEffector(self, object, parentFramePosition=(0.0,0.0,0.0), childFramePosition=(0.0,0.0,0.0)):
        """ Constrain an object to the end effector """
        return pybullet.createConstraint(
            parentBodyUniqueId=self._objectID,
            parentLinkIndex=self._endEffectorIndex,
            childBodyUniqueId=object._objectID,
            childLinkIndex=-1,
            jointType=pybullet.JOINT_FIXED,
            jointAxis=[0, 0, 0],
            parentFramePosition=parentFramePosition,
            childFramePosition=childFramePosition
        )
    
    def removeConstraintFromEndEffector(self, object, constraint):
        """ Unconstrain an object from the end effector """
        pybullet.removeConstraint(constraint)
