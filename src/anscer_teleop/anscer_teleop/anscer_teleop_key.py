"""
@file anscer_teleop_key.py
@author Abubakarsiddiq Navid shaikh
@date 2024-10-05
@brief Auto-generated author information
"""

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Robot!
---------------------------
Moving around:
       u    i    o
       j    k    l
       m    ,    .

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
space key, k : force stop
anything else : stop smoothly

CTRL-C to quit
"""

moveBindings = {
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'l':(0,-1),
        'u':(1,1),
        ',':(-1,0),
        '.':(-1,1),
        'm':(-1,-1),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

class TeleopKeyNode(Node):
    def __init__(self):
        super().__init__('anscer_teleop_key')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        self.speed = 0.5
        self.turn = 1.0
        self.x = 0
        self.th = 0
        self.status = 0
        self.count = 0
        self.accel = 0.1
        self.target_speed = 0
        self.target_turn = 0
        self.control_speed = 0
        self.control_turn = 0

        self.settings = termios.tcgetattr(sys.stdin)

    def run(self):
        try:
            print(msg)
            print(vels(self.speed,self.turn))
            while(True):
                key = getKey(self.settings)
                if key in moveBindings.keys():
                    self.x = moveBindings[key][0]
                    self.th = moveBindings[key][1]
                    self.count = 0
                elif key in speedBindings.keys():
                    self.speed = self.speed * speedBindings[key][0]
                    self.turn = self.turn * speedBindings[key][1]
                    self.count = 0
                    print(vels(self.speed,self.turn))
                    if (self.status == 14):
                        print(msg)
                    self.status = (self.status + 1) % 15
                elif key == ' ' or key == 'k' :
                    self.x = 0
                    self.th = 0
                    self.control_speed = 0
                    self.control_turn = 0
                else:
                    self.count = self.count + 1
                    if self.count > 4:
                        self.x = 0
                        self.th = 0
                    if (key == '\x03'):
                        break

                self.target_speed = self.speed * self.x
                self.target_turn = self.turn * self.th

                if self.target_speed > self.control_speed:
                    self.control_speed = min( self.target_speed, self.control_speed + self.accel )
                elif self.target_speed < self.control_speed:
                    self.control_speed = max( self.target_speed, self.control_speed - self.accel )
                else:
                    self.control_speed = self.target_speed

                if self.target_turn > self.control_turn:
                    self.control_turn = min( self.target_turn, self.control_turn + self.accel )
                elif self.target_turn < self.control_turn:
                    self.control_turn = max( self.target_turn, self.control_turn - self.accel )
                else:
                    self.control_turn = self.target_turn

                twist = Twist()
                twist.linear.x = self.control_speed
                twist.angular.z = self.control_turn
                self.publisher_.publish(twist)

        except Exception as e:
            print(e)

        finally:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
