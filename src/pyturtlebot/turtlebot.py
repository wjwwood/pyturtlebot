import sys
import time

import rospy

from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist

_turtlebot_singleton = None


def get_robot():
    global _turtlebot_singleton
    if _turtlebot_singleton is None:
        _turtlebot_singleton = Turtlebot()
    return _turtlebot_singleton


class Turtlebot(object):
    max_linear = 1.0
    max_angular = 2.0

    def __init__(self):
        rospy.init_node('pyturtlebot', anonymous=True)
        rospy.myargv(argv=sys.argv)

        self.on_bumper = None

        self.__cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
        self.__bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.__bumper_handler)

    def move(self, linear=0.0, angular=0.0):
        # Bounds checking
        if abs(linear) > self.max_linear:
            self.say("Whoa! Slowing you down to within +/-{0} m/s...".format(self.max_linear))
            linear = self.max_linear if linear > self.max_linear else linear
            linear = -self.max_linear if linear < -self.max_linear else linear
        if abs(angular) > self.max_angular:
            self.say("Whoa! Slowing you down to within +/-{0} rad/s...".format(self.max_angular))
            angular = self.max_angular if angular > self.max_angular else angular
            angular = -self.max_angular if angular < -self.max_angular else angular
        # Message generation
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        # Announce and publish
        self.say("Moving ('{linear}' m/s, '{angular}' rad/s)...".format(linear=linear, angular=angular))
        self.__cmd_vel_pub.publish(msg)

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.say("Stopping the robot!")
        self.__cmd_vel_pub.publish(msg)

    def wait(self, seconds):
        self.say("Waiting for '{0}' seconds.".format(seconds))
        time.sleep(seconds)

    def say(self, msg):
        print(msg)
        sys.stdout.flush()

    def __bumper_handler(self, msg):
        if msg.state != BumperEvent.PRESSED or msg.bumper != BumperEvent.CENTER:
            return
        if self.on_bumper is not None:
            self.on_bumper.__call__()
