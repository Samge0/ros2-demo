import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    
    current_number = 1
    
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.publish_message)

    def publish_message(self):
        msg = String()
        msg.data = f'Hello, ROS {self.current_number}'
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.publisher_.publish(msg)
        self.current_number += 1

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
