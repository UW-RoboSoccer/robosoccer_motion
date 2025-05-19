import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner_node')
        self.create_subscription(String, '/behavior_state', self.behavior_callback, 10)
        self.create_subscription(String, '/joint_state', self.joint_callback, 10)
        self.create_subscription(String, '/imu/data', self.imu_data_callback, 10)
        self.create_subscription(String, '/imu/euler_ori', self.imu_euler_callback, 10)
        self.create_subscription(String, '/left/force', self.left_force_callback, 10)
        self.create_subscription(String, '/right/force', self.right_force_callback, 10)
        self.footsteps_pub = self.create_publisher(String, '/footsteps', 10) 
        self.target_pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        self.get_logger().info('Motion planner node has started.')

        # TODO: Implement footstep and kick planning

    def behavior_callback(self, msg):
        # TODO: Plan footsteps and kicks
        pass

    def joint_callback(self, msg):
        pass

    def imu_data_callback(self, msg):
        pass

    def imu_euler_callback(self, msg):
        pass

    def left_force_callback(self, msg):
        pass

    def right_force_callback(self, msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MotionPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
