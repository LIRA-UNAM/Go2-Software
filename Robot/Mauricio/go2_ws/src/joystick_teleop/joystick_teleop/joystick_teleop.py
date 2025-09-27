import math
import rclpy
import threading
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('joystick_teleop')
        self.__pubCmdVel = self.create_publisher(Twist, '/cmd_vel', 5)
        self.__pubTrick  = self.create_publisher(String, '/go2_trick', 1)
        self.__sub = self.create_subscription(Joy,'/joy',
            self.joyCallback, 5)
        self.__busy = False
        self.__turbo = False
        self.__tmr = None
    #end def


    def __busyClear(self):
        self.__busy = False
    #end def


    def publishTrick(self, trick):
        self.__busy = True
        msgTrick = String()
        msgTrick.data = trick
        self.__pubTrick.publish(msgTrick)
        self.__tmr = threading.Timer(2.5, self.__busyClear)
        self.__tmr.start()
    #end def


    def handleButtons(self, msgJoy):
        if msgJoy.buttons[0]: # C^
            self.publishTrick("stand")
            return True
        elif msgJoy.buttons[1]: # C>
            self.publishTrick("sit")
            return True
        elif msgJoy.buttons[2]: # Cv
            self.publishTrick("lay")
            return True
        elif msgJoy.buttons[3]: # C<
            self.publishTrick("scrape")
            return True
        elif msgJoy.buttons[4]: # L
            self.publishTrick("wallow")
            return True
        # elif msgJoy.buttons[5]: # R
        elif msgJoy.buttons[7]: # Z
            self.publishTrick("scrape")
            return True
        elif msgJoy.buttons[6]: # A
            self.publishTrick("dance1")
            return True
        elif msgJoy.buttons[8]: # B
            self.publishTrick("dance2")
            return True
        elif msgJoy.axes[6] > 0: # DPad Up
            self.publishTrick("bodyUp")
        elif msgJoy.axes[6] < 0: # DPad Down
            self.publishTrick("bodyDown")
        self.__turbo = bool(msgJoy.buttons[5])
        return False
    #end def


    def joyCallback(self, msgJoy):
        handled = self.handleButtons(msgJoy)
        if handled or self.__busy:
            return

        leftStickX  = msgJoy.axes[0]
        leftStickY  = msgJoy.axes[1]
        rightStickX = msgJoy.axes[2]
        rightStickY = msgJoy.axes[3]

        lx, az = self.axis2lxaz(leftStickX, leftStickY)
        msgTwist = Twist()
        msgTwist.linear.x = lx
        msgTwist.linear.y = 0.0
        msgTwist.linear.z = 0.0
        msgTwist.angular.x = 0.0
        msgTwist.angular.y = 0.0
        msgTwist.angular.z = az
        self.__pubCmdVel.publish(msgTwist)
    #end def


    def axis2lxaz(self, x, y):
        magnitude = math.sqrt(x**2 + y**2)
        if magnitude < 0.1:
            return 0.0, 0.0
        lx = (1.6 if self.__turbo else 0.8) * y
        az = (1.2 if self.__turbo else 0.9) * x
        return lx, az
    #end def


    def xy2rtheta(self, x, y):
        r = math.sqrt(x**2 + y**2)
        theta = math.atan2(x, y)
        return r, theta
    #end def
#end class


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
