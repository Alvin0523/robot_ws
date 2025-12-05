import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TwistStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class ArduPilotOdomAdapter(Node):
    def __init__(self):
        super().__init__('ardupilot_odom_adapter')

        # --- DEFINE QOS PROFILE ---
        # This matches ArduPilot's DDS settings (Best Effort = Fire and Forget)
        qos_policy = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 1. Subscribers (Now using the compatible QoS profile)
        self.pose_sub = self.create_subscription(
            PoseStamped, 
            '/ap/pose/filtered', 
            self.pose_cb, 
            qos_policy  # <--- FIXED HERE
        )
        
        self.twist_sub = self.create_subscription(
            TwistStamped, 
            '/ap/twist/filtered', 
            self.twist_cb, 
            qos_policy  # <--- FIXED HERE
        )

        # 2. Publisher & TF (Standard Reliable is fine for outputting to Nav2)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Settings
        self.odom_frame = 'odom'
        self.base_frame = 'base_footprint' 
        self.current_twist = TwistStamped()
        self.first_packet_received = False

        self.get_logger().info('✅ ArduPilot Odom Adapter Initialized (QoS: Best Effort). Waiting for data...')

    def twist_cb(self, msg):
        self.current_twist = msg

    def pose_cb(self, msg):
        # A. Create Odometry Message
        odom = Odometry()
        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose = msg.pose
        odom.twist.twist = self.current_twist.twist

        # Publish
        self.odom_pub.publish(odom)

        # B. Broadcast TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.tf_broadcaster.sendTransform(t)

        # --- PRINT ONCE ---
        if not self.first_packet_received:
            self.get_logger().info(f'🚀 First Data Received! Publishing /odom and TF now. (X: {msg.pose.position.x:.2f})')
            self.first_packet_received = True

def main(args=None):
    rclpy.init(args=args)
    node = ArduPilotOdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()