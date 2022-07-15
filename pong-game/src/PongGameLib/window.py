from operator import imod
import cv2
import numpy as np
from gameDefintions import FieldParameters as fp

class Renderer():
    
    def __init__(self,fieldParams):
        self.__width= fieldParams.fieldWidth
        self.__height=fieldParams.fieldHeight
        self.__borderSize = fieldParams.borderSize
        self.__barLength = fieldParams.playerBarLength

    def createWindowWithBorder(self):
        self.__image = np.zeros((self.__height,self.__width,3),np.uint8)
        self.__image = cv2.rectangle(self.__image,(0,0),(self.__width,self.__height),(128,128,128),self.__borderSize)
        self.__image = cv2.line(self.__image,(int(self.__width/2),0),(int(self.__width/2),self.__height),(128,128,128),int(self.__borderSize/2))

    def setScore(self,score1,score2):
        font = cv2.FONT_HERSHEY_DUPLEX
        text = str(score1)+":"+str(score2)
        textsize = cv2.getTextSize(text, font, 2, 2)[0]
        textX = int((self.__width - textsize[0]) / 2)
        textY = 70
        cv2.putText(self.__image, text, (textX, textY ), font, 2, (255, 255, 255), 2)

    def showWindow(self):
        cv2.imshow('Image',self.__image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def drawLeftPlayerBar(self, positionY):
        self.__image = cv2.line(self.__image,(self.__borderSize,positionY-int(self.__barLength/2)),(self.__borderSize,positionY+int(self.__barLength/2)),(255,255,255),int(self.__borderSize/2))
    
    def drawRightPlayerBar(self, positionY):
        self.__image = cv2.line(self.__image,(self.__width-self.__borderSize,positionY-int(self.__barLength/2)),(self.__width-self.__borderSize,positionY+int(self.__barLength/2)),(255,255,255),int(self.__borderSize/2))

    def drawBall(self,position):
        self.__image = cv2.circle(self.__image,(int(position[0]),int(position[1])),10,(255,255,255),-1)

    def getImage(self):
        return self.__image
