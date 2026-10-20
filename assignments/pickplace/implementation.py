#
# Runs main loop
#


import cv2


OBJECT_POSITION = [0.5, 0.4, 0.8]
BIN_LEGO_POSITION   = [-0.2, 0.4, 0.9]
BIN_MUG_POSITION   = [-0.2,  0.75, 0.9]
REST_POSITION   = [0.0, 0.3, 1.2]

LEGO_CONSTRAINT_OFFSET = 0.07
MUG_CONSTRAINT_OFFSET = 0.035
LOCATION_EPS = 1e-2


class Implementation:
    def __init__(self, robot, camera, object, classifier, trueObjectLabel):
        self._robot = robot
        self._camera = camera
        self._object = object
        self._classifier = classifier
        self.predictedObjectLabel = None
        self.trueObjectLabel = trueObjectLabel
        self._elapsed = 0.0
        self._state = 'start'
        self.done = False
        self._constraint = None
    
    def onTick(self, deltaTime):
        """ This function is called on each tick of the clock
        
        :param deltaTime: time between ticks
        """
        
        if self._elapsed < 1.0 or self.done:
            self._robot.sendEndEffectorPose(REST_POSITION)
        else:
            if self._state == 'start':
                # At the start, capture an image and classify it, then move to "pickup"
                image, depth = self._camera.render()
                prediction = self._classifier.predictSingle(cv2.resize(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), (32,32)), self._classifier.weights)
                self.predictedObjectLabel = 'lego' if prediction > 0.5 else 'mug'
                self.predictionConfidence = prediction if prediction > 0.5 else 1.0 - prediction
                print('Predicted object as {} with confidence {:.1f}% (true label is {})'.format(self.predictedObjectLabel, self.predictionConfidence * 100.0, self.trueObjectLabel))
                self._state = 'pickup'
            elif self._state == 'pickup':
                # Move to the pickup position, then after that return to "rest"
                self._robot.sendEndEffectorPose(OBJECT_POSITION)
                if self._robot.distanceFromGoal() < LOCATION_EPS:
                    self._constraint = self._robot.addConstraintToEndEffector(
                        self._object,
                        parentFramePosition=(0.0,0.0,LEGO_CONSTRAINT_OFFSET if self.trueObjectLabel == 'lego' else MUG_CONSTRAINT_OFFSET),
                        childFramePosition=(0.0,0.0,0.0)
                    )
                    self._state = 'pickup-rest'
            elif self._state == 'pickup-rest':
                # Move to the rest position, then after that move to state "place"
                self._robot.sendEndEffectorPose(REST_POSITION)
                if self._robot.distanceFromGoal() < LOCATION_EPS:
                    self._state = 'place'
            elif self._state == 'place':
                # Move to the place position, then after that move to "rest"
                self._robot.sendEndEffectorPose(BIN_LEGO_POSITION if self.predictedObjectLabel == 'lego' else BIN_MUG_POSITION)
                if self._robot.distanceFromGoal() < LOCATION_EPS:
                    self._robot.removeConstraintFromEndEffector(self._object, self._constraint)
                    self._constraint = None
                    self._state = 'place-rest'
            elif self._state == 'place-rest':
                # Move to the rest position, then consider done
                self._robot.sendEndEffectorPose(REST_POSITION)
                if self._robot.distanceFromGoal() < LOCATION_EPS:
                    self._state = 'done'
            elif self._state == 'done':
                self._robot.sendEndEffectorPose(REST_POSITION)
                self.done = True
        
        self._elapsed += deltaTime
