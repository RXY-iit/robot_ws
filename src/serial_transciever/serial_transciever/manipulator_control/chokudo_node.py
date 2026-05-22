import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class ChokudoNode(Node):
    """Simple node to perform direct (chokudo) moves.

    Subscribes to `/chokudo_angles` (Float32MultiArray). When a message is
    received the node republishes the same float array to `/motor_angles`.
    This keeps direct-move logic isolated in a small node.
    """

    def __init__(self):
        super().__init__('chokudo_node')
        self.pub = self.create_publisher(Float32MultiArray, '/motor_angles', 10)
        self.sub = self.create_subscription(
            Float32MultiArray,
            '/chokudo_angles',
            self._chokudo_cb,
            10,
        )
        self.get_logger().info('Chokudo node started. Publish Float32MultiArray to /chokudo_angles to move directly.')

    def _chokudo_cb(self, msg: Float32MultiArray):
        # Basic validation: must contain numeric data
        if not hasattr(msg, 'data') or len(msg.data) == 0:
            self.get_logger().warn('Received empty /chokudo_angles message; ignoring')
            return
        out = Float32MultiArray()
        try:
            out.data = [float(x) for x in msg.data]
        except Exception as e:
            self.get_logger().warn(f'Invalid data in /chokudo_angles: {e}')
            return
        self.pub.publish(out)
        self.get_logger().info(f'Chokudo: published direct angles ({len(out.data)} values)')


def main(args=None):
    rclpy.init(args=args)
    node = ChokudoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
