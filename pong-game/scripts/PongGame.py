#!/usr/bin/env python
import ImagePublisher as imp
from PongGameLib import gameDefintions, window, playerBar, ball, score
import intera_interface
import rospy

def main():
    #Creating Objects
    publisher = imp.ImagePublisher(20)
    field = gameDefintions.FieldParameters()
    screen = window.Renderer(field)
    nav = intera_interface.Navigator()
    barRightPlayer = playerBar.PositionCalculator(field,nav.get_wheel_state('right_wheel'))
    barLeftPlayer = playerBar.PositionCalculator(field,nav.get_wheel_state('head_wheel'))
    gameBall = ball.PositionCalculator(field) 
    scoreStore = score.Store()
    #Create Field and Bars with ball
    while(True):
        screen.createWindowWithBorder()
        PositionRightPlayer = barRightPlayer.getActualPosition(nav.get_wheel_state('right_wheel'))
        PositionLeftPlayer = barLeftPlayer.getActualPosition(nav.get_wheel_state('head_wheel'))
        screen.drawLeftPlayerBar(PositionLeftPlayer)
        screen.drawRightPlayerBar(PositionRightPlayer)
        screen.drawBall(gameBall.getPosition())
        gameSituation = gameBall.moveBall(PositionRightPlayer,PositionLeftPlayer)
        if scoreStore.storePossiblePlayerPoint(gameSituation):
            gameBall.resetBall()
        screen.setScore(scoreStore.scoreLeftPlayer,scoreStore.scoreRightPlayer)
        publisher.publishImage(screen.getImage())

    
if __name__ == '__main__':
    #try:
    main()
    #except rospy.ROSInterruptException:
     #   pass



