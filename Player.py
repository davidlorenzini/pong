from Settings import Settings, Direction

class Player(Settings):
    def __init__(self, controller=None):
        Settings.__init__(self)
        self._actual_position = int(self.window_height/2)

        if controller:
            self.controller = controller
            self.state = self.controller.get_state()

    def move_player(self):
        new_state = self.controller.get_state()
        if new_state == 0 and self.state == 255:
            self._actual_position += self.bar_speed
        elif new_state == 255 and self.state == 0:
            self._actual_position -= self.bar_speed
        elif new_state < self.state:
            self._actual_position -= self.bar_speed
        elif new_state > self.state:
            self._actual_position += self.bar_speed
        
        self.state = new_state
        self._check_position()
        return self._actual_position

    def move_player_auto(self, position_y):
        if self._actual_position < position_y - int(self.bar_speed/2):
            self._actual_position += Direction.DOWN * int(self.bar_speed/2)

        elif self._actual_position > position_y + int(self.bar_speed/2):
            self._actual_position += Direction.UP * int(self.bar_speed/2)
            
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
