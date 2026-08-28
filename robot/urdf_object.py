import pybullet
import numpy as np


class URDFObject:
    def __init__(self, urdf, position=(0.0,0.0,0.0), rotation=(0.0,0.0,0.0,1.0), fixedBase=False, globalScaling=1.0):
        """ Create a URDF object
        
        :param urdf: urdf file path
        :param position: position (xyz)
        :param rotation: rotation (xyzw)
        :param fixedBase: if True, the base is considered fixed (not moving)
        :param globalScaling: scale of object
        """
        self._objectID = pybullet.loadURDF(urdf, position, rotation, useFixedBase=fixedBase, globalScaling=globalScaling)

    def getBaseTransform(self):
        """ Returns transform of object in form (position, rotation), where position is xyz and rotation is xyzw """
        return pybullet.getBasePositionAndOrientation(self._objectID)
    
    def resetBaseTransform(self, position=(0.0,0.0,0.0), rotation=(0.0,0.0,0.0,1.0)):
        """ Resets object to the provided position and rotation """
        pybullet.resetBasePositionAndOrientation(self._objectID, position, rotation)

    def getBaseWorldVelocity(self):
        """ Returns tuple of linear and angular velocity of base (worldspace) """
        return pybullet.getBaseVelocity(self._objectID)
    
    def getDynamicsInfo(self):
        """ Return the dynamics info (mass, com, inertia, etc.) of robot """
        dynamics = pybullet.getDynamicsInfo(self._objectID, -1)
        return {
            'mass': dynamics[0],
            'lateral_friction': dynamics[1],
            'local_inertia_diag': dynamics[2],
            'local_inertial_position': dynamics[3],
            'local_inertial_orientation': dynamics[4],
            'restitution': dynamics[5],
            'rolling_friction': dynamics[6],
            'spinning_friction': dynamics[7],
            'contact_damping': dynamics[8],
            'contact_stiffness': dynamics[9],
            'body_type': dynamics[10],
            'collision_margin': dynamics[11]
        }

    def getBaseLocalVelocity(self):
        """ Returns tuple of linear and angular velocity of base (localspace) """
        robotWorldPosition, robotWorldRotation = self.getBaseTransform()
        robotWorldRotationM = np.array(pybullet.getMatrixFromQuaternion(robotWorldRotation)).reshape(3, 3)
        linearVelocityWorld, angularVelocityWorld = self.getBaseWorldVelocity()
        linearVelocityRobot = tuple((robotWorldRotationM.T @ linearVelocityWorld).tolist())
        angularVelocityRobot = tuple((robotWorldRotationM.T @ angularVelocityWorld).tolist())
        return linearVelocityRobot, angularVelocityRobot
    
    def resetBaseVelocity(self, linear=(0.0, 0.0, 0.0), angular=(0.0, 0.0, 0.0)):
        """ Reset the base's velocity (worldspace) """
        pybullet.resetBaseVelocity(self._objectID, linear, angular)

    def setBaseColor(self, rgba=(1,1,1,1)):
        """ Set the base color to the provided RGBA color """
        pybullet.changeVisualShape(self._objectID, -1, rgbaColor=rgba)

    def applyForce(self, linkIndex, force, position, frame='world'):
        """ Apply a force at given link
        
        :param linkIndex: index of link
        :param force: force (x,y,z)
        :param position: position to apply force (x,y,z)
        :param frame: world or link
        """
        if frame not in ['world', 'link']:
            raise ValueError("Frame '{}' not valid for applying force".format(frame))
        pybullet.applyExternalForce(self._objectID, linkIndex, force, position, pybullet.WORLD_FRAME if frame == 'world' else pybullet.LINK_FRAME)

    def applyTorque(self, linkIndex, torque, position, frame='world'):
        """ Apply a torque at given link
        
        :param linkIndex: index of link
        :param torque: torque (x,y,z)
        :param position: position to apply torque (x,y,z)
        :param frame: world or link
        """
        if frame not in ['world', 'link']:
            raise ValueError("Frame '{}' not valid for applying force".format(frame))
        pybullet.applyExternalForce(self._objectID, linkIndex, torque, position, pybullet.WORLD_FRAME if frame == 'world' else pybullet.LINK_FRAME)
    
    def getJoints(self):
        """ Retrieve the joints of the object """
        numJoints = pybullet.getNumJoints(self._objectID)
        joints = []
        for jointIndex in range(numJoints):
            jointInfoPb = pybullet.getJointInfo(self._objectID, jointIndex)
            joints.append({
                'index': jointInfoPb[0],
                'name': str(jointInfoPb[1].decode('ascii')),
                'type': jointInfoPb[2],
                'qIndex': jointInfoPb[3],
                'uIndex': jointInfoPb[4],
                'damping': jointInfoPb[6],
                'friction': jointInfoPb[7],
                'lowerLimit': jointInfoPb[8],
                'upperLimit': jointInfoPb[9],
                'maxForce': jointInfoPb[10],
                'maxVelocity': jointInfoPb[11],
                'linkName': str(jointInfoPb[12].decode('ascii')),
                'parentLinkIndex': jointInfoPb[16],
            })
        return joints
    
    def getJointIndexFromName(self, jointName):
        """ Find the joint index from a joint name """
        joints = self.getJoints()
        for joint in joints:
            if joint['name'] == jointName:
                return joint['index']
        return -1

    def getJointState(self, jointIndex):
        """ Get state of joint """
        jointStatePb = pybullet.getJointState(self._objectID, jointIndex)
        return {
            'position': jointStatePb[0],
            'velocity': jointStatePb[1],
            'reactionForces': jointStatePb[2],
            'appliedMotorTorque': jointStatePb[3],
        }
    
    def getLinkState(self, linkIndex):
        """ Get state of a link """
        linkStatePb = pybullet.getLinkState(self._objectID, linkIndex, computeLinkVelocity=1, computeForwardKinematics=1)
        return {
            'worldPosition': linkStatePb[0],
            'worldRotation': linkStatePb[1],
            'localInertialFramePosition': linkStatePb[2],
            'localInertialFrameRotation': linkStatePb[3],
            'worldLinkFramePosition': linkStatePb[4],
            'worldLinkFrameRotation': linkStatePb[5],
            'worldLinkLinearVelocity': linkStatePb[6],
            'worldLinkAngularVelocity': linkStatePb[7],
        }
    
    def resetJointState(self, jointIndex, targetPosition, targetVelocity=None):
        """ Reset state of joint """
        kwargs = {
            'jointIndex': jointIndex,
            'targetValue': targetPosition
        }
        if targetVelocity is not None:
            kwargs['targetVelocity'] = targetVelocity
        pybullet.resetJointState(self._objectID, **kwargs)

    def setJointMotor(self, jointIndex, controlMode, targetPosition=None, targetVelocity=None, force=None, positionGain=None, velocityGain=None, maxVelocity=None):
        """ Set joint motor

        :param jointIndex: index of joint
        :param controlMode: position or velocity
        :param targetPosition: target position (float)
        :param targetVelocity: target velocity (float)
        :param force: force (float)
        :param positionGain: position gain (float)
        :param velocityGain: velocity gain (float)
        :param maxVelocity: max velocity (float)
        """
        if controlMode == 'position':
            controlModePb = pybullet.POSITION_CONTROL
        elif controlMode == 'velocity':
            controlModePb = pybullet.VELOCITY_CONTROL
        else:
            raise ValueError("Invalid control mode '{}'".format(controlMode))

        kwargs = {
            'jointIndex': jointIndex,
            'controlMode': controlModePb,
        }
        if targetPosition is not None:
            kwargs['targetPosition'] = targetPosition
        if targetVelocity is not None:
            kwargs['targetVelocity'] = targetVelocity
        if force is not None:
            kwargs['force'] = force
        if positionGain is not None:
            kwargs['positionGain'] = positionGain
        if velocityGain is not None:
            kwargs['velocityGain'] = velocityGain
        if maxVelocity is not None:
            kwargs['maxVelocity'] = maxVelocity
        
        pybullet.setJointMotorControl2(self._objectID, **kwargs)

    def setJointMotors(self, jointIndices, controlMode, targetPositions=None, targetVelocities=None, forces=None, positionGains=None, velocityGains=None, maxVelocities=None):
        """ Set joint motors

        :param jointIndices: indices of joints
        :param controlMode: position or velocity
        :param targetPositions: target positions (float)
        :param targetVelocities: target velocities (float)
        :param forces: forces (float)
        :param positionGains: position gains (float)
        :param velocityGains: velocity gains (float)
        """
        if controlMode == 'position':
            controlModePb = pybullet.POSITION_CONTROL
        elif controlMode == 'velocity':
            controlModePb = pybullet.VELOCITY_CONTROL
        else:
            raise ValueError("Invalid control mode '{}'".format(controlMode))

        kwargs = {
            'jointIndices': jointIndices,
            'controlMode': controlModePb,
        }
        if targetPositions is not None:
            kwargs['targetPositions'] = targetPositions
        if targetVelocities is not None:
            kwargs['targetVelocities'] = targetVelocities
        if forces is not None:
            kwargs['forces'] = forces
        if positionGains is not None:
            kwargs['positionGains'] = positionGains
        if velocityGains is not None:
            kwargs['velocityGains'] = velocityGains
        if maxVelocities is not None:
            kwargs['maxVelocities'] = maxVelocities
        
        pybullet.setJointMotorControlArray(self._objectID, **kwargs)

    def tick(self, deltaTime):
        """ Tick """
        pass
