import cv2 
import numpy as np
from gameDefintions import FieldParameters as fp

class Renderer:
    
    def __init__(self, fieldParams: fp):
        self._width = fieldParams.fieldWidth
        self._height = fieldParams.fieldHeight
        self._borderSize = fieldParams.borderSize
        self._barLength = fieldParams.playerBarLength

    def createWindowWithBorder(self):
        self._image = np.zeros((self._height, self._width, 3), np.uint8)
        self._image = cv2.rectangle(
            self._image, 
            (0, 0), 
            (self._width, self._height), 
            (128, 128, 128),
            self._borderSize
        )
        self._image = cv2.line(
            self._image, 
            (int(self._width / 2), 0), 
            (int(self._width / 2), self._height), 
            (128, 128, 128), 
            int(self._borderSize / 2)
        )

    def setScore(self, score1: int, score2: int):
        font = cv2.FONT_HERSHEY_DUPLEX
        text = str(score1) + ":" + str(score2)
        textsize = cv2.getTextSize(text, font, 2, 2)[0]
        textX = int((self._width - textsize[0]) / 2)
        textY = 70
        cv2.putText(self._image, text, (textX,  textY ), font, 2, (255, 255, 255), 2)

    def showWindow(self):
        cv2.imshow('Image', self._image)
        cv2.waitKey(3)
    
    def closeWindow(self):
        cv2.destroyAllWindows()

    def drawLeftPlayerBar(self,  positionY: int):
        self._image = cv2.line(
            self._image, 
            (self._borderSize, positionY - int(self._barLength / 2)), 
            (self._borderSize, positionY + int(self._barLength / 2)), 
            (255, 255, 255), 
            int(self._borderSize / 2)
        )
    
    def drawRightPlayerBar(self,  positionY: int):
        self._image = cv2.line(
            self._image, 
            (self._width - self._borderSize, positionY - int(self._barLength / 2)),
            (self._width - self._borderSize, positionY + int(self._barLength / 2)), 
            (255, 255, 255), 
            int(self._borderSize / 2)
        )

    def drawBall(self, position: list[int]):
        self._image = cv2.circle(
            self._image, 
            (int(position[0]), int(position[1])), 
            10, 
            (255, 255, 255), 
            -1
        )

    def getImage(self):
        return self._image
