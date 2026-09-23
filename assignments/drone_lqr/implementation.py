#
# This is the file you will be modifying.
#


import numpy as np
import scipy.linalg
import scipy.spatial.transform


class Implementation:
    def __init__(self, robot, controlMethod='openloop', gravity=9.8, desiredPosition=(0.0, 0.0, 1.0)):
        self._robot = robot
        self._elapsed = 0.0
        self._controlMethod = controlMethod
        self._gravity = gravity
        self._desiredPosition = desiredPosition
        self._controlBound = 4 * (self._robot.mass * self._gravity / 4.0) # Per rotor
        self.lastDesiredState = None
        self.lastActualState = None
        self._cachedRiccatiSolutions = [] # each is tuple with ((A,B,Q,R), P)

    def solveContinuousARE(self, A, B, Q, R):
        """ Solve the continuous algebriac Riccati equation
        
        DO NOT MODIFY

        Uses a cached implementation to enable faster computation

        :param A: A matrix as numpy array
        :param B: B matrix as numpy array
        :param Q: Q matrix as numpy array
        :param R: R matrix as numpy array
        :returns: Riccati equation solution matrix P as numpy array
        """
        for c in self._cachedRiccatiSolutions:
            err = np.array([
                np.abs(np.array(A) - np.array(c[0][0])).max(),
                np.abs(np.array(B) - np.array(c[0][1])).max(),
                np.abs(np.array(Q) - np.array(c[0][2])).max(),
                np.abs(np.array(R) - np.array(c[0][3])).max()
            ]).max()
            if err < 1e-7:
                return c[1]
        P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        self._cachedRiccatiSolutions.append(((A,B,Q,R), P))
        return P

    def getDesiredState(self):
        """ Returns the desired state of the quadrotor
        
        DO NOT MODIFY

        :returns: desired state of quadrotor as numpy array
        """
        return np.array(list(self._desiredPosition) + [0.0]*9)

    def controlLoop(self, deltaTime):
        """ Runs the control loop of the quadrotor
        
        DO NOT MODIFY

        :param deltaTime: delta time between ticks
        """
        if self._controlMethod == 'openloop':
            control = self.computeControlOpenLoop()
        elif self._controlMethod == 'lqr':
            desiredState = self.getDesiredState()
            currentState = self.getRobotState()
            A = self.getSystemMatrix()
            B = self.getInputMatrix()
            Q, R = self.getLQRCostMatrices()
            control = self.computeControlLQR(currentState=currentState, desiredState=desiredState, A=A, B=B, Q=Q, R=R)
        else:
            raise ValueError("Invalid control method!")
        # Clamp controls
        control = np.clip(control, min=-self._controlBound, max=self._controlBound)
        self._robot.applyRotorCommands(control)

    # Your implementation!
    # -----------------------------------------------------------------------
    def computeControlOpenLoop(self):
        """ Compute open loop controls to have quadrotor hover, assuming it starts in equilibrium (stationary, level flight)
        
        :returns: list of forces for each rotor [f1, f2, f3, f4] as a numpy array.
            f1 is rotor in +y direction, f2 is rotor in +x direction, f3 is rotor in -y direction, and f4 is rotor in -x direction
        """
        # These may be helpful
        mass = self._robot.mass
        gravity = self._gravity
        # TODO:
        # Your answer here
        return np.array([0.0, 0.0, 0.0, 0.0])

    def getRobotState(self):
        """ Return the state vector for the robot
        
        The state should be 12-dimensional, and should be composed of:
            (p_x, p_y, p_z, theta_x, theta_y, theta_z, v_x, v_y, v_z, omega_x, omega_y, omega_z)
        
        The position and orientation should be in global frame.
        The velocities should be in robot body frame.

        :returns: robot state (12-dimensional) as a numpy array
        """
        # These may be helpful
        position, rotation = self._robot.getBaseTransform()
        eulerAngles = scipy.spatial.transform.Rotation.from_quat(rotation).as_euler('xyz') # same result as pybullet.getEulerFromQuaternion
        linearVelocity, angularVelocity = self._robot.getBaseLocalVelocity()
        # TODO:
        # Your answer here
        return np.array([0.0]*12)

    def getSystemMatrix(self):
        """ Returns the system matrix A of the linearized quadrotor model

        Recall: in a linear state-space representation, we have x'=Ax+Bu.
        This function should return the A matrix in this state space representation.
        
        :returns: the A matrix, which should be a numpy array of shape (12, 12)
        """
        # This may be helpful
        gravity = self._gravity
        # TODO:
        # Your answer here
        return np.zeros((12,12))

    def getInputMatrix(self):
        """ Returns the input matrix B of the linearized quadrotor model
        
        Recall: in a linear state-space representation, we have x'=Ax+Bu.
        This function should return the B matrix in this state space representation.

        :returns: the B matrix, which should be a numpy array of shape (12, 4)
        """
        # These may be helpful
        gravity = self._gravity
        mass = self._robot.mass
        drag = self._robot.drag
        rotorArmLength = self._robot.armLength
        robotInertiaIx = self._robot.inertia[0,0]
        robotInertiaIy = self._robot.inertia[1,1]
        robotInertiaIz = self._robot.inertia[2,2]
        # TODO:
        # Your answer here
        return np.zeros((12,4))

    def getLQRCostMatrices(self):
        """ Returns the cost matrices for LQR
        
        The cost matrices are the Q and R matrices that are defined in the cost function that LQR optimizes.

        For each cost matrix, you can use a simple diagonal matrix.
        You have freedom to choose these matrices; pick ones that show desirable behavior for stabilizing the quadrotor.

        :returns: a tuple containing the Q and R matrices; these matrices should be numpy arrays of shape (12,12) and (4,4), respectively.
        """
        # TODO:
        # Your answer here
        Q = np.zeros((12,12))
        R = np.zeros((4,4))
        return (Q, R)

    def computeControlLQR(self, currentState, desiredState, A, B, Q, R):
        """ Compute controls that stabilize the quadcopter to hover using LQR.
        
        Using LQR, you should compute needed controls [f1, f2, f3, f4], which are the forces on each of the rotors.
            f1 is rotor in +y direction, f2 is rotor in +x direction, f3 is rotor in -y direction, and f4 is rotor in -x direction

        For LQR, you should assume a infinite-horizon, continuous-time formulation.
        While technically our system is discrete and finite time, the time step is small and the time horizon is large, so this approximation is reasonably good.

        For solving the Riccati equation, use the function: `self.solveContinuousARE`

        :param currentState: current state of quadrotor as numpy array
        :param desiredState: desired state of quadrotor as numpy array (should be equilibrium)
        :param A: state space A matrix as numpy array with shape (12,12)
        :param B: state space B matrix as numpy array with shape (12,4)
        :param Q: LQR Q matrix as numpy array with shape (12,12)
        :param R: LQR R matrix as numpy array with shape (4,4)
        :returns: list of forces for each rotor [f1, f2, f3, f4] as a numpy array.
        """
        # These may be helpful
        mass = self._robot.mass
        gravity = self._gravity
        # TODO:
        # Your answer here
        return np.array([0.0, 0.0, 0.0, 0.0])
    # -----------------------------------------------------------------------

    def onTick(self, deltaTime):
        """ This function is called on each tick of the clock
        
        :param deltaTime: time between ticks
        """

        # Control the robot
        # -----------------------------------------------------------
        self.controlLoop(deltaTime)
        # -----------------------------------------------------------

        self.lastDesiredState = self.getDesiredState()
        self.lastActualState = self.getRobotState()
        self._elapsed += deltaTime
