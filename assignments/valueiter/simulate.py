import pybullet
import numpy as np
from robot.simulation import Simulation
from robot.urdf_object import URDFObject
from robot.r2d2 import R2D2
from assignments.valueiter.gridworld import Action, CellType
from enum import Enum


ANGLE_THRESHOLD = 0.1
POSITION_THRESHOLD = 0.1
MAX_LINEAR_SPEED = 0.8
MAX_ANGULAR_SPEED = 0.8
POSITION_DAMP = 0.25
YAW_DAMP = 0.25


class CONTROL_STATE(Enum):
    WAITING = 0
    REORIENT = 1
    MOVE = 2
    DONE = 3


class GridWorldSimulation:
    def __init__(self, world, results, startPosition=(0.0,0.0,0.0), startRotation=(0.0,0.0,0.0,1.0), blockSize=1.5):
        """ Grid world simulation object

        :param world: grid world
        :param results: value iteration results
        :param startPosition: starting position for robot
        :param startRotation: starting rotation for robot
        :param blockSize: the size of each block in the grid world
        """
        self._world = world
        self._results = results
        self._blockSize = blockSize
        self._controlTarget = {'position': None, 'angle': None}
        self.done = False
        self._controlState = CONTROL_STATE.WAITING
        self._floor = URDFObject("plane.urdf", position=(0.0,0.0,-0.1), fixedBase=True)
        self._tiles = []
        for rowIndex, row in enumerate(world.world):
            for colIndex, cell in enumerate(row):
                verticalOffset = 0.0
                if cell.type == CellType.HAZARD:
                    color = np.array([255,0,0], dtype=np.uint8)
                    verticalOffset = 0.5
                elif cell.type == CellType.SLIPPERY:
                    color = np.array([0,0,255], dtype=np.uint8)
                elif cell.type == CellType.GOAL:
                    color = np.array([0,255,0], dtype=np.uint8)
                else:
                    color = np.array([200,200,200], dtype=np.uint8)

                tile = URDFObject(
                    "cube.urdf",
                    position=[rowIndex*blockSize, colIndex*blockSize, -blockSize/2.0 + verticalOffset],
                    rotation=pybullet.getQuaternionFromEuler([0, 0, 0]),
                    fixedBase=True,
                    globalScaling=blockSize
                )
                tile.setBaseColor(np.concatenate([color, np.array([255])]).astype(np.float32)/255.0)
                self._tiles.append(tile)
        self._robot = R2D2(
            position=(startPosition[0]*blockSize, startPosition[1]*blockSize, startPosition[2]+0.5),
            rotation=startRotation
        )
        self._robot.setJointMotor(8, 'position', -0.35) # Set arm to retracted

    def sendCommand(self, linear, angular):
        """ Send a command to the robot """
        self._robot.sendCommand(linear, angular)

    def getRobotVelocity(self):
        """ Get the noisy velocity of the robot (in robot frame)

        :returns: tuple of 2 np arrays (each of length 3) corresponding to linear and angular velocity
        """
        linearVelocity, angularVelocity = self._robot.getBaseLocalVelocity()
        linearVelocity = np.array(linearVelocity) + np.random.normal(loc=0.0, scale=0.3, size=(3,))
        angularVelocity = np.array(angularVelocity) + np.random.normal(loc=0.0, scale=0.6, size=(3,))
        return linearVelocity, angularVelocity
    
    def getGroundtruthRobotPose(self):
        """ Get the groundtruth position of the robot (in world frame)

        :returns: tuple of 2 np.arrays (length 3 and a 3x3) corresponding to position and rotation matrix
        """
        robotWorldPosition, robotWorldRotation = self._robot.getBaseTransform()
        robotWorldRotationM = np.array(pybullet.getMatrixFromQuaternion(robotWorldRotation)).reshape(3, 3)
        return np.array(robotWorldPosition), robotWorldRotationM

    def positionToCell(self, x, y):
        """ Convert a 2D position (x,y) to a grid world cell
        
        :param x: x coordinate
        :param y: y coordinate
        :returns: (row, col)
        """
        return (
            int(np.clip(np.round(x/self._blockSize).astype(int), 0, self._world.height-1)),
            int(np.clip(np.round(y/self._blockSize).astype(int), 0, self._world.width-1))
        )
    
    def cellToPosition(self, row, col):
        """ Convert a grid world cell to a world position (x, y)

        :param row: row
        :param col: col
        :returns: (x, y) as np.array
        """
        return np.array([
            row*self._blockSize,
            col*self._blockSize
        ], dtype=float)
    
    def controlLoop(self):
        """ Run the control loop """
        # Get the location and type of current cell
        currentCellLocation = self.positionToCell(*self.getGroundtruthRobotPose()[0][0:2].tolist())
        currentCellType = self._world.world[currentCellLocation[0]][currentCellLocation[1]].type
        # State machine
        if self._controlState == CONTROL_STATE.WAITING:
            if currentCellType == CellType.GOAL:
                self._controlState = CONTROL_STATE.DONE
            else:
                action = self._results['policy'][currentCellLocation[0],currentCellLocation[1]]
                if action == Action.UP:
                    angleTarget = np.pi / 2.0
                elif action == Action.LEFT:
                    angleTarget = np.pi
                elif action == Action.DOWN:
                    angleTarget = 3.0 * np.pi / 2.0
                elif action == Action.RIGHT:
                    angleTarget = 0.0
                else:
                    self.sendCommand(0.0, 0.0)
                    self.done = True
                    return
                self._controlTarget = {'position': None, 'angle': angleTarget}
                self._controlState = CONTROL_STATE.REORIENT
        elif self._controlState == CONTROL_STATE.REORIENT:
            robotRotationM = self.getGroundtruthRobotPose()[1]
            robotYaw = float(np.arctan2(robotRotationM[1,0], robotRotationM[0,0]))
            yawError = self._controlTarget['angle'] - robotYaw
            yawError = float(np.arctan2(np.sin(yawError), np.cos(yawError)))
            if abs(yawError) > ANGLE_THRESHOLD:
                # Simple P control
                self.sendCommand(0.0, float(np.clip(yawError, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)))
            else:
                # Send stop command and move to next state in state machine
                self.sendCommand(0.0, 0.0)
                action = self._results['policy'][currentCellLocation[0],currentCellLocation[1]]
                if action == Action.UP:
                    positionTarget = self.cellToPosition(currentCellLocation[0]-1, currentCellLocation[1])
                elif action == Action.LEFT:
                    positionTarget = self.cellToPosition(currentCellLocation[0], currentCellLocation[1]-1)
                elif action == Action.DOWN:
                    positionTarget = self.cellToPosition(currentCellLocation[0]+1, currentCellLocation[1])
                elif action == Action.RIGHT:
                    positionTarget = self.cellToPosition(currentCellLocation[0], currentCellLocation[1]+1)
                else:
                    positionTarget = None
                self._controlTarget = {'position': positionTarget, 'angle': None}
                self._controlState = CONTROL_STATE.MOVE
        elif self._controlState == CONTROL_STATE.MOVE:
            robotPosition, robotRotationM = self.getGroundtruthRobotPose()
            robotYaw = float(np.arctan2(robotRotationM[1,0], robotRotationM[0,0]))
            positionTarget = np.array(self._controlTarget['position'])
            angleTarget = np.arctan2(-(positionTarget[0] - robotPosition[0]), positionTarget[1] - robotPosition[1])
            yawError = angleTarget - robotYaw
            yawError = float(np.arctan2(np.sin(yawError), np.cos(yawError)))
            positionError = np.linalg.norm(positionTarget - robotPosition[:2])
            # Simple P control for angle and position
            if abs(positionError) > POSITION_THRESHOLD:
                self.sendCommand(
                    float(MAX_LINEAR_SPEED * np.clip((positionError**POSITION_DAMP), 0, 1) * np.clip(abs(1.0 - np.clip(yawError/np.pi, 0, 1))**YAW_DAMP, 0, 1)), # The exponentiation is to damp it a bit
                    float(np.clip(yawError, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED))
                )
            else:
                self._controlTarget = {'position': None, 'angle': None}
                self._controlState = CONTROL_STATE.WAITING
                self.sendCommand(0.0, 0.0)
        else:
            self.sendCommand(0.0, 0.0)
            self.done = True
    
    def tick(self, deltaTime):
        """ Tick the grid world implementation """
        self.controlLoop()
        self._robot.tick(deltaTime)


def simulateGridWorld(world, results, startPosition=(0.0,0.0,0.0), startRotation=(0.0,0.0,0.0,1.0), blockSize=1.5, DELTA_TIME=1.0/240.0, MAX_TIME_SECONDS=90.0, headless=False):
    """ Simulate the value iteration results in PyBullet """

    pyBulletSimulation = Simulation(DELTA_TIME, headless=headless, realtime=False if headless else True)
    pybullet.resetDebugVisualizerCamera(
        cameraDistance=20.0,
        cameraYaw=45,
        cameraPitch=-30,
        cameraTargetPosition=startPosition
    )
    gridWorldSimulation = GridWorldSimulation(world, results, startPosition, startRotation, blockSize)

    currentTime = 0.0
    for i in range(int(MAX_TIME_SECONDS / DELTA_TIME)):
        if currentTime > 1.0:
            # Main functionality after a second to prevent issues with initialization
            gridWorldSimulation.tick(DELTA_TIME)
        if gridWorldSimulation.done:
            break
        pyBulletSimulation.step()
        currentTime += DELTA_TIME
    pyBulletSimulation.cleanup()
