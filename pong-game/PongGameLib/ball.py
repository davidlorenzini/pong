import numpy as np
import random
import math
from gameDefintions import GameAction, ReflectionType, FieldParameters as fp

class PositionCalculator:
    
    def __init__(self, fieldParams: fp):
        self._barLength = fieldParams.playerBarLength
        self._height = fieldParams.fieldHeight
        self._width = fieldParams.fieldWidth
        self._angle =  self._setRandomAngle()
        self._borderSize = fieldParams.borderSize
        self._speedIncreaser = 2
        self._speed = self._speedStart = 10
        self._actualX = int(self._width / 2)
        self._actualY = int(self._height / 2)

    def _setRandomAngle(self):
        angle =  random.random() * (math.pi / 2) - (math.pi / 4)
        if int(random.random() * 2) == 1:
            return angle
        
        return angle + math.pi

    def moveBall(self, positionRightPlayer: int, positionLeftPlayer: int):
        #Move first
        self._actualX += math.cos(self._angle) * self._speed
        self._actualY += math.sin(self._angle) * self._speed
        
        #Check possible Reflection
        reflection = self._checkReflection()
        
        #No Reflection, Game continues
        
        if reflection[0] == type(ReflectionType.NoReflection):
            return GameAction.GameContinue
        
        #Vertical Reflection, Game continues but angel must be changed
        elif reflection[0] == ReflectionType.VerticalReflection:
            self._modifyAngle(reflection[1])
            self._speed = self._speed + self._speedIncreaser
            return GameAction.GameContinue
        
        #Player Reflection, angel is changed if reflection is there otherwise one Player loose
        elif reflection[0] == ReflectionType.PossibleLeftPlayerReflection:
            if self._checkReflectionOfPlayer(positionLeftPlayer):
                self._modifyAngle(reflection[1])
                self._speed = self._speed + self._speedIncreaser
                return GameAction.GameContinue
            return GameAction.RightPlayerScored
        
        elif reflection[0] == ReflectionType.PossibleRightPlayerReflection:
            if self._checkReflectionOfPlayer(positionRightPlayer):
                self._modifyAngle(reflection[1])
                self._speed = self._speed + self._speedIncreaser
                return GameAction.GameContinue
            return GameAction.LeftPlayerScored

    def _checkReflectionOfPlayer(self, positionPlayer: int):
        if positionPlayer - int(self._barLength / 2) < self._actualY < positionPlayer + int(self._barLength / 2):
            return True
        return False

    def _modifyAngle(self, normalVector: list[int]):
        #All Vectors are used as if they were centred on [0, 0]
        #Equation is r = d - 2d * n / norm(n) * n
        d = 1 * [math.cos(self._angle), math.sin(self._angle)]
        r = d - 2 * (np.dot(d, normalVector) / np.dot(normalVector, normalVector) * np.array(normalVector))
        
        #New angle
        self._angle = math.atan2(r[1], r[0])

    def _checkReflection(self) -> tuple[ReflectionType, list[int]]:
        case = ReflectionType.NoReflection
        normalVector = [0, 0]
        
        if self._actualY < self._borderSize:
            normalVector = [0, -1]
            self._actualY = self._borderSize
            case = ReflectionType.VerticalReflection
        
        elif self._actualY > self._height - self._borderSize:
            normalVector = [0, 1]
            self._actualY = self._height - self._borderSize
            case = ReflectionType.VerticalReflection
        
        elif self._actualX < 2 * self._borderSize:
            normalVector = [-1, 0]
            self._actualX = 2 * self._borderSize
            case = ReflectionType.PossibleLeftPlayerReflection
        
        elif self._actualX > self._width - 2 * self._borderSize:
            normalVector = [1, 0]
            self._actualX = self._width - 2 * self._borderSize
            case = ReflectionType.PossibleRightPlayerReflection
        
        return (case, normalVector)

    def resetBall(self):
        self._angle = self._setRandomAngle()
        self._actualX = int(self._width / 2)
        self._actualY = int(self._height / 2)
        self._speed = self._speedStart

    def getPosition(self):
        return [self._actualX, self._actualY]