import numpy as np
import random
import math
from gameDefintions import GameAction, ReflectionType, FieldParameters as fp

class PositionCalculator():
    
    def __init__(self,fieldParams):
        self.__barLength = fieldParams.playerBarLength
        self.__height = fieldParams.fieldHeight
        self.__width = fieldParams.fieldWidth
        self.__angle =  self._setRandomAngle()
        self.__borderSize = fieldParams.borderSize
        self.__speedIncreaser = 2
        self.__speed = self.__speedStart = 10
        self.__actualX = int(self.__width/2)
        self.__actualY = int(self.__height/2)

    def _setRandomAngle(self):
        angle =  random.random()*(math.pi/2)-(math.pi/4)
        if int(random.random()*2) == 1:
            return angle
        return angle + math.pi

    def moveBall(self,positionRightPlayer,positionLeftPlayer):
        #Move first
        self.__actualX += math.cos(self.__angle)*self.__speed
        self.__actualY += math.sin(self.__angle)*self.__speed
        #Check possible Reflection
        reflection = self._checkReflection()
        #No Reflection, Game continues
        if reflection[0] == type(ReflectionType.NoReflection):
            return GameAction.GameContinue
        #Vertical Reflection, Game continues but angel must be changed
        elif reflection[0] == ReflectionType.VerticalReflection:
            self._modifyAngle(reflection[1])
            self.__speed=self.__speed+self.__speedIncreaser
            return GameAction.GameContinue
        #Player Reflection, angel is changed if reflection is there otherwise one Player loose
        elif reflection[0]==ReflectionType.PossibleLeftPlayerReflection:
            if (self._checkReflectionOfPlayer(positionLeftPlayer)):
                self._modifyAngle(reflection[1])
                self.__speed=self.__speed+self.__speedIncreaser
                return GameAction.GameContinue
            return GameAction.RightPlayerScored
        elif reflection[0]==ReflectionType.PossibleRightPlayerReflection:
            if (self._checkReflectionOfPlayer(positionRightPlayer)):
                self._modifyAngle(reflection[1])
                self.__speed=self.__speed+self.__speedIncreaser
                return GameAction.GameContinue
            return GameAction.LeftPlayerScored

    def _checkReflectionOfPlayer(self,positionPlayer):
        if positionPlayer-int(self.__barLength/2)<self.__actualY<positionPlayer+int(self.__barLength/2):
            return True
        return False

    def _modifyAngle(self,normalVector):
        #All Vectors are used as if they were centred on [0,0]
        #Equation is r = d-2d*n/norm(n)*n
        d=1*[math.cos(self.__angle),math.sin(self.__angle)]
        r=d-2*(np.dot(d,normalVector)/np.dot(normalVector,normalVector)*np.array(normalVector))
        #New angle
        self.__angle = math.atan2(r[1],r[0])

    def _checkReflection(self):
        case = ReflectionType.NoReflection
        normalVector = [0,0]
        if self.__actualY<self.__borderSize:
            normalVector=[0,-1]
            self.__actualY=self.__borderSize
            case= ReflectionType.VerticalReflection
        elif self.__actualY>self.__height-self.__borderSize:
            normalVector=[0,1]
            self.__actualY=self.__height-self.__borderSize
            case = ReflectionType.VerticalReflection
        elif self.__actualX<2*self.__borderSize:
            normalVector=[-1,0]
            self.__actualX=2*self.__borderSize
            case = ReflectionType.PossibleLeftPlayerReflection
        elif self.__actualX>self.__width-2*self.__borderSize:
            normalVector=[1,0]
            self.__actualX=self.__width-2*self.__borderSize
            case = ReflectionType.PossibleRightPlayerReflection
        return (case,normalVector)

    def resetBall(self):
        self.__angle =   self._setRandomAngle()
        self.__actualX = int(self.__width/2)
        self.__actualY = int(self.__height/2)
        self.__speed = self.__speedStart

    def getPosition(self):
        return [self.__actualX,self.__actualY]