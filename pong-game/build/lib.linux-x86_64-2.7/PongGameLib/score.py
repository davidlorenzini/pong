from gameDefintions import GameAction 

class Store():
    def __init__(self):
        self.scoreRightPlayer = 0
        self.scoreLeftPlayer = 0

    def storePossiblePlayerPoint(self,action):
        if action == GameAction.LeftPlayerScored:
            self.scoreLeftPlayer=self.scoreLeftPlayer+1
            return True
        if action == GameAction.RightPlayerScored:
            self.scoreRightPlayer=self.scoreRightPlayer+1
            return True
        return False
