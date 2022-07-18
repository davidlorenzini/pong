from Settings import Settings, Direction

class Player(Settings):
    def __init__(self, nav=None):
        Settings.__init__(self)
        self._actual_position = int(self.window_height/2)

        if nav:
            self.nav = nav
            self.wheel_state = self.nav.get_wheel_state()

    def move_player(self):
        if not hasattr(self, "nav"):
            return self._actual_position
        
        new_wheel_state = self.nav.get_wheel_state()
        if new_wheel_state < self.wheel_state:
            self._actual_position -= self.bar_speed
        elif new_wheel_state > self.wheel_state:
            self._actual_position += self.bar_speed
        
        self.wheel_state = new_wheel_state
        self._check_position()
        return self._actual_position

    def move_player_auto(self, position_y):
        if self._actual_position < position_y - self.bar_speed:
            self._actual_position += Direction.DOWN * self.bar_speed

        elif self._actual_position > position_y + self.bar_speed:
            self._actual_position += Direction.UP * self.bar_speed
            
        self._check_position()
        return self._actual_position

    def _check_position(self):
        if int(self._actual_position + self.bar_height/2) > self.window_height:
            self._actual_position = int(self.window_height - self.bar_height/2)
        elif int(self._actual_position - self.bar_height/2) < 0:
            self._actual_position = int(self.bar_height/2)

    def get_position(self):
        return self._actual_position

    def reset_player(self):
        self._actual_position = int(self.window_height/2)
