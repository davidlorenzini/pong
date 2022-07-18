from Settings import Settings, Direction

class Player(Settings):
    def __init__(self):
        super().__init__()
        self._actual_position = int(self.bar_height/2)

    def move_player(self, direction: Direction) -> int:
        self._actual_position += direction * self.bar_speed
        self._check_position()
        return self._actual_position

    def move_player_auto(self, position_y: int) -> int:
        if self._actual_position < position_y - self.bar_speed:
            return self.move_player(Direction.DOWN)
        elif self._actual_position > position_y + self.bar_speed:
            return self.move_player(Direction.UP)
        return self.move_player(Direction.NONE)

    def _check_position(self):
        if int(self._actual_position + self.bar_height/2) > self.window_height:
            self._actual_position = int(self.window_height - self.bar_height/2)
        elif int(self._actual_position - self.bar_height/2) < 0:
            self._actual_position = int(self.bar_height/2)

    def get_position(self):
        return self._actual_position

    def reset_player(self):
        self._actual_position = int((self.window_height - self.bar_height)/2)
