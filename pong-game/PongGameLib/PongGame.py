#!/usr/bin/env python
import gameDefintions, window, playerBar, ball, score
import time
# import intera_interface 
# import rospy

def main():
    #Creating Objects
    field = gameDefintions.FieldParameters()
    pos = (field.fieldHeight - field.playerBarLength)/2
    screen = window.Renderer(field)
    # nav = intera_interface.Navigator()
    barRightPlayer = playerBar.PositionCalculator(field, pos)
    barLeftPlayer = playerBar.PositionCalculator(field, pos)
    gameBall = ball.PositionCalculator(field) 
    scoreStore = score.Store()
    #Create Field and Bars with ball
    count = 0
    while count < 500 :
        # print(count)
        screen.createWindowWithBorder()

        PositionRightPlayer = barRightPlayer.getActualPosition(pos)
        PositionLeftPlayer = barLeftPlayer.getActualPosition(pos)

        screen.drawLeftPlayerBar(PositionLeftPlayer)
        screen.drawRightPlayerBar(PositionRightPlayer)
        screen.drawBall(gameBall.getPosition())

        gameSituation = gameBall.moveBall(PositionRightPlayer, PositionLeftPlayer)
        if scoreStore.storePossiblePlayerPoint(gameSituation):
            gameBall.resetBall()
        screen.setScore(scoreStore.scoreLeftPlayer, scoreStore.scoreRightPlayer)
        screen.showWindow()
        count += 1
        time.sleep(0.05)
        # publisher.publishImage(screen.getImage())
    screen.closeWindow()

    
if __name__ == '__main__':
    #try:
    main()
    #except rospy.ROSInterruptException:
     #   pass



