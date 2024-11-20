import rclpy, time
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty, String


class TelloJoy(Node):

    def __init__(self):
        super().__init__('tello_joy')
        self.subscription = self.create_subscription(Joy, '/joy_filtered', self.callback, 1)
        self.subscription  # prevent unused variable warning

        self.pub_control   = self.create_publisher(Twist,  '/control', 10)
        self.pub_emergency = self.create_publisher(Empty,  '/emergency', 1)
        self.pub_takeoff   = self.create_publisher(Empty,  '/takeoff', 1)
        self.pub_land      = self.create_publisher(Empty,  '/land', 1)
        self.pub_flip      = self.create_publisher(String, '/flip', 1)

        self.linear_scale  = 100
        self.angular_scale = 100

    def callback(self, msg):
        vel = Twist()
        flip = String()
        empty_msg = Empty()

        vel.linear.x = msg.axes[2]  * self.linear_scale
        vel.linear.y = -msg.axes[3]  * self.linear_scale
        vel.linear.z = msg.axes[1]  * self.linear_scale
        vel.angular.z = -msg.axes[0] * self.angular_scale

        btn = msg.buttons

        #self.get_logger().info(str(msg.axes))

        if btn[9]:
            self.pub_takeoff.publish(empty_msg)
            self.get_logger().info('Take off - Start Pressed')
            time.sleep(1)
        elif btn[8]:
            self.pub_land.publish(empty_msg)
            self.get_logger().info('Land - Back Pressed')
            time.sleep(1)
        elif btn[4] and btn[5] and btn[6] and btn[7]:
            self.pub_emergency.publish(empty_msg)
            self.get_logger().info('Emergency - 4 Shoulders')
            time.sleep(1)
        elif btn[0]:
            flip.data = "l"
            self.get_logger().info('Flip Left - X Pressed')
            self.pub_flip.publish(flip)
            time.sleep(1)
        elif btn[2]:
            flip.data = "r"
            self.get_logger().info('Flip right - B Pressed')
            self.pub_flip.publish(flip)
            time.sleep(1)
        elif btn[1]:
            flip.data = "b"
            self.get_logger().info('Flip back - A Pressed')
            self.pub_flip.publish(flip)
            time.sleep(1)
        elif btn[3]:
            flip.data = "f"
            self.get_logger().info('Flip front - Y Pressed')
            self.pub_flip.publish(flip)
            time.sleep(1)

        self.pub_control.publish(vel)

        #print(vel.linear.x,vel.angular.z)

def main(args=None):
    rclpy.init(args=args)

    tello_joy = TelloJoy()

    rclpy.spin(tello_joy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    tello_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()