from enum import Enum 

class FieldParameters:
    def __init__(self):
        self.fieldHeight = 600
        self.fieldWidth = 1024
        self.borderSize = 30
        self.playerBarLength = 100

class ReflectionType(Enum):
    NoReflection = 0
    VerticalReflection = 1
    PossibleRightPlayerReflection = 2
    PossibleLeftPlayerReflection = 3

class GameAction(Enum):
    GameContinue = 0
    RightPlayerScored = 1
    LeftPlayerScored = 2