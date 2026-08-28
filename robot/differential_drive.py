import pybullet
import numpy as np


MAX_STALE_TIME = 1.0
MAX_MOTOR_VELOCITY = 50.0
MAX_MOTOR_VELOCITY_CHANGE = 25.0


class DifferentialDrive:
    def __init__(
        self,
        urdfObject,
        leftWheelJoints,
        rightWheelJoints,
        wheelSeparation,
        wheelRadius,
        wheelSeparationMultiplier=1.0,
        wheelRadiusMultiplier=1.0,
        kpLinear=100.0,
        kdLinear=1.0,
        kpAngular=75.0,
        kdAngular=1.0
    ):
        self._urdfObject = urdfObject
        self._leftWheelJoints = leftWheelJoints
        self._rightWheelJoints = rightWheelJoints
        self._wheelSeparation = wheelSeparation
        self._wheelRadius = wheelRadius
        self._wheelSeparationMultiplier = wheelSeparationMultiplier
        self._wheelRadiusMultiplier = wheelRadiusMultiplier

        self._desiredLinear = 0.0
        self._desiredAngular = 0.0

        self._lastLeft = 0.0
        self._lastRight = 0.0

        self._kpLinear = kpLinear
        self._kdLinear = kdLinear

        self._kpAngular = kpAngular
        self._kdAngular = kdAngular

        self._lastErrLinear = 0.0
        self._lastErrAngular = 0.0

        self._staleTime = 0.0

    def sendCommand(self, linear, angular):
        """ Send linear and angular velocity command """
        self._desiredLinear = linear
        self._desiredAngular = angular
        self._staleTime = 0.0

    def tick(self, deltaTime):
        """ Tick the controller """
        measuredLinear, measuredAngular = self.computeBodyVelocity()

        left, right = self.computeWheelTargets(deltaTime, measuredLinear, measuredAngular)
        left, right = self.applyRateLimit(left, right, deltaTime)
        left, right = self.normalize(left, right)

        self.sendToMotors(left, right)

        self._lastLeft  = left
        self._lastRight = right

        self._staleTime += deltaTime
        if self._staleTime > MAX_STALE_TIME:
            self._desiredLinear = 0.0
            self._desiredAngular = 0.0

    def computeBodyVelocity(self):
        """ Compute the velocity in body frame """
        lin, ang = self._urdfObject.getBaseLocalVelocity()
        return float(lin[1]), float(ang[2])

    def computeWheelTargets(self, dt, measuredLinear, measuredAngular):
        """ Compute target wheel velocities """
        adjustedSeparation = self._wheelSeparation * self._wheelSeparationMultiplier
        adjustedRadius     = self._wheelRadius * self._wheelRadiusMultiplier

        # Errors
        errorLinear  = self._desiredLinear - measuredLinear
        errorAngular = self._desiredAngular - measuredAngular

        # Derivatives
        derivLinear  = (errorLinear - self._lastErrLinear) / dt
        derivAngular = (errorAngular - self._lastErrAngular) / dt

        self._lastErrLinear  = errorLinear
        self._lastErrAngular = errorAngular

        # PD control
        controlLinear  = self._desiredLinear + self._kpLinear * errorLinear + self._kdLinear * derivLinear
        controlAngular = self._desiredAngular + self._kpAngular * errorAngular + self._kdAngular * derivAngular

        left  = -(controlLinear - controlAngular * adjustedSeparation / 2.0) / adjustedRadius
        right = -(controlLinear + controlAngular * adjustedSeparation / 2.0) / adjustedRadius

        return left, right

    def normalize(self, left, right):
        """ Normalize wheel velocities, but doing so in a way that maintains ratio """
        maxMagnitude = max(abs(left), abs(right))
        if maxMagnitude > MAX_MOTOR_VELOCITY:
            scale = MAX_MOTOR_VELOCITY / maxMagnitude
            left *= scale
            right *= scale
        return left, right

    def applyRateLimit(self, left, right, dt):
        """ Apply a limit to the rate of change in wheel velocities """
        deltaLeft  = left  - self._lastLeft
        deltaRight = right - self._lastRight

        maxDelta = MAX_MOTOR_VELOCITY_CHANGE * dt

        scale = max(1.0, abs(deltaLeft) / maxDelta, abs(deltaRight) / maxDelta)

        left  = self._lastLeft + deltaLeft / scale
        right = self._lastRight + deltaRight / scale

        return left, right

    def sendToMotors(self, left, right):
        """ Send wheel velocities to motors """
        joints = self._urdfObject.getJoints()

        leftIdx = [j['index'] for j in joints if j['name'] in self._leftWheelJoints]
        rightIdx = [j['index'] for j in joints if j['name'] in self._rightWheelJoints]

        self._urdfObject.setJointMotors(leftIdx, 'velocity', targetVelocities=[left] * len(leftIdx))
        self._urdfObject.setJointMotors(rightIdx, 'velocity', targetVelocities=[right] * len(rightIdx))
