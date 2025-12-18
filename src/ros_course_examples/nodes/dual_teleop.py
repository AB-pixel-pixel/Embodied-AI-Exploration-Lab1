#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

msg = """
Control Your Turtlebot!
---------------------------
Robot 1 (tb3_0):    Robot 2 (tb3_1):
   w                   ^
 a s d               < v >

Space: Force Stop Both
CTRL-C to quit
"""

moveBindings_1 = {
    'w': (1, 0, 0, 0),
    'a': (0, 0, 0, 1),
    'd': (0, 0, 0, -1),
    's': (-1, 0, 0, 0),
}

moveBindings_2 = {
    '\x1b[A': (1, 0, 0, 0), # Up
    '\x1b[D': (0, 0, 0, 1), # Left
    '\x1b[C': (0, 0, 0, -1), # Right
    '\x1b[B': (-1, 0, 0, 0), # Down
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 0.5
turn = 1.0

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed,turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('dual_teleop')
    pub1 = rospy.Publisher('/tb3_0/cmd_vel', Twist, queue_size=1)
    pub2 = rospy.Publisher('/tb3_1/cmd_vel', Twist, queue_size=1)

    x1 = 0; th1 = 0; status = 0
    x2 = 0; th2 = 0

    try:
        print(msg)
        while(1):
            key = getKey()
            
            # Robot 1 Controls
            if key in moveBindings_1.keys():
                x1 = moveBindings_1[key][0]
                th1 = moveBindings_1[key][3]
            elif key == ' ' or key == 'k' :
                x1 = 0; th1 = 0
                x2 = 0; th2 = 0
            else:
                x1 = 0; th1 = 0
                if (key == '\x03'):
                    break

            # Robot 2 Controls (Arrow keys send sequences)
            # The getKey function logic for arrows is simplified above
            # But let's check if the key matches the sequence directly
            
            if key in moveBindings_2.keys():
                x2 = moveBindings_2[key][0]
                th2 = moveBindings_2[key][3]
            elif key == ' ': # Handled above
                pass
            else:
                # If key was just 'w', x2 should be 0.
                # But since we read one key at a time, we need to be careful not to reset R2 when R1 moves
                # However, this simple blocking read approach makes simultaneous control hard on one keyboard.
                # A better approach is to toggle state or use a non-blocking check that maintains state.
                # For this simple demo, we will reset if the key doesn't match R2 bindings
                # BUT, this means if I press 'w', R2 stops. If I press 'Up', R1 stops.
                # To support "simultaneous" feel, we need to track state or use separate keys without auto-stop.
                # Let's try to maintain state for a short duration? 
                # No, standard teleop stops when key is released (or in this case, when loop iterates without key).
                # To fix this, we will use a timeout. If no key, stop.
                # But here we are processing one key.
                
                # Logic update:
                # We received 'key'.
                # Is it for R1? Update target_twist1.
                # Is it for R2? Update target_twist2.
                # If 'key' is empty (timeout), stop both? Or keep going?
                # Standard teleop keeps going if you hold it (system sends repeats).
                
                pass 
            
            # Re-eval for R2 because the "else" above was tricky
            if key in moveBindings_2.keys():
                x2 = moveBindings_2[key][0]
                th2 = moveBindings_2[key][3]
            elif key in moveBindings_1.keys():
                # If it was an R1 key, R2 should stop? 
                # Ideally yes, unless we have a complex key state manager.
                # For this version 1, "Shared Keyboard", it's usually turn-based or rapid switching.
                x2 = 0; th2 = 0
            else:
                x2 = 0; th2 = 0

            # Correct logic for R1 if it was R2 key
            if key in moveBindings_2.keys():
                x1 = 0; th1 = 0

            # Publish R1
            twist1 = Twist()
            twist1.linear.x = x1 * speed
            twist1.linear.y = 0
            twist1.linear.z = 0
            twist1.angular.x = 0
            twist1.angular.y = 0
            twist1.angular.z = th1 * turn
            pub1.publish(twist1)

            # Publish R2
            twist2 = Twist()
            twist2.linear.x = x2 * speed
            twist2.linear.y = 0
            twist2.linear.z = 0
            twist2.angular.x = 0
            twist2.angular.y = 0
            twist2.angular.z = th2 * turn
            pub2.publish(twist2)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        pub1.publish(twist)
        pub2.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
