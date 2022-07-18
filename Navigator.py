import intera_interface
import rospy

class Navigator:
    def __init__(self, button = "right"):
        #rospy.init_node("Nav_wheel", anonymous=True)
        self.nav = intera_interface.Navigator()
        self.wheel = button +"_wheel"

    def get_wheel_state(self):
        return self.nav.get_wheel_state(self.wheel)

    

    
    