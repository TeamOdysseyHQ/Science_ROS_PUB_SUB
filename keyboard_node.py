import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import sys
import termios
import tty

class KeyboardPublisher(Node):

    def __init__(self):
        super().__init__('science_keyboard_controller')
        self.pub = self.create_publisher(Int32MultiArray, '/science_control', 10)
        self.get_logger().info("Keyboard controller started")

    def get_key(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            # tty.setraw(fd)
            tty.setcbreak(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch

    def run(self):
        msg = Int32MultiArray()
        # LA => -1 = go down, 1 = go up,
        # Drill => , -2 = clockwise, -1 = decrease speed, 0 = abrupt stop, 1 = increase speed, 2 = counter-clockwise
        # barrel_rotate =>-1 = rotate backward, 1 = rotate forward
        # servo_toggle => 0 = off servo, 1 = on servo, 2 = drop sample
        # science_explore_toggle => 0= off science module, 1 = On science exploration
        msg.data = [-6, -6, -6, -6, -6]  # [linear Actuator, drill, barrel_rotate, servo_toggle, science_explore_toggle]
        ph_servo_pressed = False
        science_mode_enabled = False
        while rclpy.ok():
            key = self.get_key()

            msg.data = [-6, -6, -6, -6, -6]

            if key == '\x1b':  # Arrow keys
                k2 = self.get_key()
                k3 = self.get_key()

                if k3 == 'A':      # UP, LA up
                    msg.data[0] = 1
                elif k3 == 'B':    # DOWN, LA down
                    msg.data[0] = -1
                elif k3 == 'C':    # RIGHT, Drill increase speed
                    msg.data[1] = 1
                elif k3 == 'D':    # LEFT, Drill decrease speed
                    msg.data[1] = -1

            # # LA - micro steps
            # elif key == '1':
            #     msg.data[0] = 0
            # elif key == '2':
            #     msg.data[0] = 4
            # elif key == '3':
            #     msg.data[0] = 8
            # elif key == '4':
            #     msg.data[0] = 16

            # # Barrel rotate - micro steps
            # elif key == '5':
            #     msg.data[2] = 0
            # elif key == '6':
            #     msg.data[2] = 4
            # elif key == '7':
            #     msg.data[2] = 8
            # elif key == '8':
            #     msg.data[2] = 16

            elif key == 'a': # abrupt stop drill
                msg.data[1] = 0
            elif key == 'q': # drill clockwise
                msg.data[1] = -2
            elif key == 'e': # drill counter-clockwise
                msg.data[1] = 2

            elif key == 'f': # barrel rotate frontward
                msg.data[2] = 1
            elif key == 'b': # barrel rotate backward
                msg.data[2] = -1

            elif key == 's': # ph servo toggle
                if not ph_servo_pressed: # not already pressed before
                    msg.data[3] = 1
                else:
                    msg.data[3] = 0
                ph_servo_pressed = not ph_servo_pressed

            elif key == 'd': # drop sample
                msg.data[3] = 2

            elif key == 'y': # science mode toggle
                if not science_mode_enabled: # not already pressed before
                    msg.data[4] = 1
                else:
                    msg.data[4] = 0
                science_mode_enabled = not science_mode_enabled

            self.pub.publish(msg)
            self.get_logger().info(f"Published: {msg.data}")

def main():
    rclpy.init()
    node = KeyboardPublisher()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
