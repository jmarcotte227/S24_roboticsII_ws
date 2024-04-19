import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped


class returnNode(Node):

    def __init__(self):
        super().__init__('return_node')
        self.pose_publisher_ = self.create_publisher(PoseStamped, 'goal_pose')
        self.obj_subscriber_ = self.create_subscription(PoseStamped, 'detected_object_pose', self.relay_pose_callback)

    def relay_pose_callback(self,msg):
        start_message = PoseStamped()
        start_message.header.frame_id = 'map'
        start_message.pose.position.x = 0.0
        start_message.pose.position.y = 0.0
        self.pose_publisher_.publish(start_message)

def main(args=None):
    rclpy.init(args=args)

    return_node = returnNode()

    rclpy.spin(return_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    return_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()