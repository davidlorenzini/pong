try:
    import intera_interface
    import rospy
except ModuleNotFoundError:
    pass

class Navigator:
    def __init__(self, button = "right"):
        #rospy.init_node("Nav_wheel", anonymous=True)
        self.nav = intera_interface.Navigator()
        self.wheel = button +"_wheel"

    def get_state(self):
        return self.nav.get_wheel_state(self.wheel)

class Keyboard:
    def __init__(self, up_key="w", down_key="s"):
        # Like Nav wheel goes from 0 to 255
        self.state = 127
        self.up_key = up_key
        self.down_key = down_key

    def key_pressed(self, key):
        if chr(key) == "q":
            return True
        elif chr(key) == self.up_key:
            self.state -= 1
        elif chr(key) == self.down_key:
            self.state += 1

    def get_state(self):
        return self.state
