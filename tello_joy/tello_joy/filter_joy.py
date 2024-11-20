import rclpy, time
from rclpy.node import Node

from sensor_msgs.msg import Joy


class FilterJoy(Node):

    def __init__(self):
        super().__init__('filter_joy')
        self.subscription = self.create_subscription(Joy, '/joy', self.callback, 10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(Joy, '/joy_filtered', 10)
        self.has_calibrated = False

        self.axes = None
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def callback(self, msg):
        self.axes    = msg.axes
        #self.axes[2] = -self.axes[2]/2 + 0.5
        #self.axes[5] = -self.axes[5]/2 + 0.5
        self.buttons = msg.buttons
    
    def timer_callback(self):
        msg = Joy()
        if self.axes is not None:
            msg.axes    = self.axes
            msg.buttons = self.buttons

            if not self.has_calibrated or all(msg.buttons[0:4]):
                self.has_calibrated = True
                self.default_axes = self.axes
                self.get_logger().info('Calibrated')
                self.pressed = False
            
            if any(msg.buttons) != 0 or msg.axes != self.default_axes:
                self.pressed = True
                self.publisher.publish(msg)
                #print(self.buttons,self.axes)
            elif self.pressed:
                self.pressed = False
                self.publisher.publish(msg)
                #print(self.buttons,self.axes)

def main(args=None):
    rclpy.init(args=args)

    filter_joy = FilterJoy()

    rclpy.spin(filter_joy)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    filter_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()