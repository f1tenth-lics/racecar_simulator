#!/usr/bin/env python3

import rospy
import sys
import select
import termios
import tty

from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Header

keyBindings = {'\x41': (1.0, 0.0),  # move forward (Up Arrow)
               '\x43': (1.0, -1.0),  # move forward and right (Right Arrow)
               '\x44': (1.0, 1.0),  # move forward and left (Left Arrow)
               '\x42': (-1.0, 0.0),  # move reverse (Down Arrow)
               'q': (0.0, 0.0)}  # all stop (q)

speed_limit = 4.0
angle_limit = 0.325


def getKey():
    tty.setraw(sys.stdin.fileno())
    key = None
    while True:
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        if key is not None:
            break
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return 'currently:\tspeed {}\tturn {}'.format(speed, turn)


if __name__ == '__main__':
    print('Use Arrow keys to move the car. Press q to stop the car.')

    settings = termios.tcgetattr(sys.stdin)
    command_pub = rospy.Publisher('/{}/vesc/low_level/ackermann_cmd_mux/input/teleop'.format(str(sys.argv[1])),
                                  AckermannDriveStamped, queue_size=1)
    rospy.init_node('keyboard_teleop', anonymous=True)

    speed = 0.0
    angle = 0.0
    status = 0.0

    try:
        while True:
            key = getKey()
            if key in keyBindings.keys():
                speed = keyBindings[key][0]
                angle = keyBindings[key][1]
            else:
                speed = 0.0
                angle = 0.0
                if (key == '\x03'):
                    break
            command = AckermannDriveStamped()
            command.header = Header()
            command.drive.speed = speed * speed_limit
            command.drive.steering_angle = angle * angle_limit
            command_pub.publish(command)

    except:
        print('raise exception: key binding error')

    finally:
        command = AckermannDriveStamped()
        command.header = Header()
        command.drive.speed = speed * speed_limit
        command.drive.steering_angle = angle * angle_limit
        command_pub.publish(command)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
