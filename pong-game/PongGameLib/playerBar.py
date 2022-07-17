from gameDefintions import FieldParameters as fp
 
class PositionCalculator:

    def __init__(self, fieldParams: fp, firstWheelPosition):
        self._maxPosition = fieldParams.fieldHeight
        self._barLength = fieldParams.playerBarLength
        self._borderSize = fieldParams.borderSize
        self._wheelPosition = firstWheelPosition
        self._speed = 60
        self._actualPosition = int(self._barLength / 2)

    def getActualPosition(self, actualWheelPosition):
        if actualWheelPosition < self._wheelPosition:
            self._actualPosition -= self._speed
        
        elif actualWheelPosition > self._wheelPosition:
            self._actualPosition += self._speed
        self._checkPosition()
        self._wheelPosition = actualWheelPosition
        
        return self._actualPosition
        
    def _checkPosition(self):
        if (self._actualPosition < self._barLength / 2 + self._borderSize):
            self._actualPosition = int(self._barLength / 2 + self._borderSize)
        
        elif (self._actualPosition > int(self._maxPosition - self._barLength / 2 - self._borderSize)):
            self._actualPosition = int(self._maxPosition - self._barLength / 2 - self._borderSize)