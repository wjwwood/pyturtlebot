import sys
import time

import rospy

from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist


class Turtlebot(object):
    def __init__(self):
        rospy.init_node('pyturtlebot', anonymous=True)
        rospy.myargv(argv=sys.argv)

        self.on_bumper = None

        self.__cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist)
        self.__bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.__bumper_handler)

    def move(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.say("Moving robot at '{0}' and '{1}', linear and angular velocities.".format(linear, angular))
        self.__cmd_vel_pub.publish(msg)

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.say("Stopping robot!")
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
