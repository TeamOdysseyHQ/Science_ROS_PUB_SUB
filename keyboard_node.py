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
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return ch

    def run(self):
        msg = Int32MultiArray()
        # LA => -1 = go down, 0 = full step, 1 = go up, 4 = 1/4 step, 8 = 1/8 step, 16 = 1/16 step
        # Drill => , -2 = clockwise, -1 = decrease speed, 0 = abrupt stop, 1 = increase speed, 2 = counter-clockwise
        # barrel_rotate => 1 = rotate right, 0 = full step, 4 = 1/4 step, 8 = 1/8 step, 16 = 1/16 step
        # servo_toggle => 1 = toggle servo
        # science_explore_toggle => 1 = toggle science exploration
        msg.data = [0, 0, 0, 0, 0]  # [linear Actuator, drill, barrel_rotate, servo_toggle, science_explore_toggle]

        while rclpy.ok():
            key = self.get_key()

            msg.data = [0, 0, 0, 0, 0]

            if key == '\x1b':  # Arrow keys
                k2 = self.get_key()
                k3 = self.get_key()

                if k3 == 'A':      # UP
                    msg.data[0] = 1
                elif k3 == 'B':    # DOWN
                    msg.data[0] = -1
                elif k3 == 'C':    # RIGHT
                    msg.data[1] = 1
                elif k3 == 'D':    # LEFT
                    msg.data[1] = -1

            elif key == 'b':
                msg.data[2] = 1

            elif key == 's':
                msg.data[3] = 1

            elif key == 'e':
                msg.data[4] = 1

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
