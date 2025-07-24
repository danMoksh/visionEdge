#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# --- START OF CONFIGURATION ---

# 1. Set your desired motor speeds
LEFT_MOTOR_FWD_SPEED = 0.50
RIGHT_MOTOR_FWD_SPEED = 0.69

# 2. IMPORTANT: Measure your robot's track width (distance between wheel centers) in meters
# Example: If the distance is 25cm, enter 0.25
TRACK_WIDTH = 0.36 # <-- CHANGE THIS VALUE

# --- END OF CONFIGURATION ---


# --- Automatic Calculation (Do not change) ---
# Calculate the linear and angular velocities needed to achieve the desired motor speeds
linear_vel = (LEFT_MOTOR_FWD_SPEED + RIGHT_MOTOR_FWD_SPEED) / 2.0
if TRACK_WIDTH == 0:
    rospy.logerr("Track width is set to 0! Please measure your robot and update the script.")
    rospy.logerr("Exiting to prevent division by zero.")
    sys.exit(1)
angular_vel = (RIGHT_MOTOR_FWD_SPEED - LEFT_MOTOR_FWD_SPEED) / TRACK_WIDTH
# ---


msg = """
Control Your Robot with Custom Speeds!
---------------------------
        w
   a    s    d
        x

'w', 'x', 's' will use your custom forward/backward speeds.
'a', 'd' will use the standard turning speeds below.

q/z : increase/decrease standard speeds by 10%
space key, s : force stop
CTRL-C to quit
"""

moveBindings = {
        # 'w', 's', 'x' are handled separately below
        'a':(0,0,0,0,0,1),  # Turn Left
        'd':(0,0,0,0,0,-1),  # Turn Right
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
    }

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard_custom')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)

    try:
        print(msg)
        print("Calculated Linear Velocity: {:.3f} m/s".format(linear_vel))
        print("Calculated Angular Velocity: {:.3f} rad/s".format(angular_vel))

        while(1):
            key = getKey()
            twist = Twist()

            if key in ('w', 'x', 's'):
                if key == 'w':
                    twist.linear.x = linear_vel
                    twist.angular.z = angular_vel
                else: # 's' or 'x' for backward
                    twist.linear.x = -linear_vel
                    twist.angular.z = -angular_vel

            elif key in moveBindings.keys():
                # Use standard speed/turn for a/d keys
                twist.linear.x = 0
                twist.angular.z = moveBindings[key][5] * turn

            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]
                print("Standard Speed: {:.2f}, Standard Turn: {:.2f}".format(speed, turn))

            elif key == ' ' :
                # Stop on spacebar, handled by the finally block
                pass

            else:
                if (key == '\x03'):
                    break

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist() # Ensure robot stops on exit
        twist.linear.x = 0; twist.angular.z = 0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

