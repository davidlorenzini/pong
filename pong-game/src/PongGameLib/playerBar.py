from gameDefintions import FieldParameters as fp

class PositionCalculator():

    def __init__(self,fieldParams,firstWheelPosition):
        self.__maxPosition = fieldParams.fieldHeight
        self.__barLength = fieldParams.playerBarLength
        self.__borderSize = fieldParams.borderSize
        self.__wheelPosition = firstWheelPosition
        self.__speed = 60
        self.__actualPosition=int(self.__barLength/2)

    def getActualPosition(self,actualWheelPosition):
        if (actualWheelPosition<self.__wheelPosition):
            self.__actualPosition -= self.__speed
        elif (actualWheelPosition>self.__wheelPosition):
            self.__actualPosition += self.__speed
        self._checkPosition()
        self.__wheelPosition = actualWheelPosition
        return self.__actualPosition
        
    def _checkPosition(self):
        if (self.__actualPosition<self.__barLength/2+self.__borderSize):
            self.__actualPosition = int(self.__barLength/2+self.__borderSize)
        elif (self.__actualPosition>int(self.__maxPosition-self.__barLength/2-self.__borderSize)):
            self.__actualPosition = int(self.__maxPosition-self.__barLength/2-self.__borderSize)