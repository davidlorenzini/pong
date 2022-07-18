class Score:
    def __init__(self):
        self.player_left = 0
        self.player_right = 0

    def player_left_scored(self):
        self.player_left += 1
        print("Left player scored\n  Score is now {}:{}".format(*self.get_score()))
    
    def player_right_scored(self):
        self.player_right += 1
        print("Right player scored\n  Score is now {}:{}".format(*self.get_score()))

    def get_score(self):
        return (self.player_left, self.player_right)
