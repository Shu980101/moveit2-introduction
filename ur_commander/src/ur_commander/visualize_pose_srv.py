import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from geometry_msgs.msg import PoseArray


class VisualizePoseSrv:
    def __init__(self):
        super().__init__("visualize_pose_srv")
        node = Node("visualize_pose_srv")
        self.publisher = node.create_publisher(
            msg_type=PoseArray,
            topic="/pose_visualizer",
            qos_profile=QoSProfile(
                durability=QoSDurabilityPolicy.VOLATILE,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
            ),
        )

        self.pose_visualizer_service = node.create_service(
            PoseArray,
            "/pose_visualizer",
            self.visualize_pose_callback,
        )
