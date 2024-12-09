import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from copy import deepcopy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from std_msgs.msg import String

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')

        # 初始化导航器
        self.navigator = BasicNavigator()

        # 等待Nav2处于活动状态
        self.navigator.waitUntilNav2Active()

        # 初始化位置
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id = 'map'
        self.initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        self.initial_pose.pose.position.x = 0.0
        self.initial_pose.pose.position.y = 0.0
        self.initial_pose.pose.orientation.w = 1.0
        self.navigator.setInitialPose(self.initial_pose)

        # 订阅目标位置话题
        self.target_subscription = self.create_subscription(
            Float32MultiArray,
            '/target_position',
            self.target_callback,
            30
        )
        self.publisher = self.create_publisher(String, 'navigate_over', 10)        # 创建话题，告知导航结束开始夹取

        # TF 变换监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 用于存储目标位置的变量
        self.target_position = None
        self.navigating = False  # 标志位，表示是否已在导航

    def target_callback(self, msg):
        """目标位置订阅回调"""
        if len(msg.data) == 3:
            self.target_position = {
                'x': msg.data[0],
                'y': msg.data[1],
                'z': msg.data[2]
            }
            if self.navigating == False:
                # self.navigating = True
                # self.get_logger().info(f"Received target position: {self.target_position}")
                self.navigate_to_target()

    def navigate_to_target(self):
        """执行导航到目标点"""
        if self.target_position is None:
            self.get_logger().error("No target position received!")
            return

        # 获取 base_link 相对于 map 坐标系的变换
        try:
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            except:
                return
            self.get_logger().info(f"Base link transform: {transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}")

            # 计算目标点相对于机器人的 yaw 角度（单位：弧度）
            yaw = math.atan2(self.target_position['y'], self.target_position['x'])

            # 使用 scipy 将 yaw 转换为四元数 (欧拉角顺序为 ZYX, 这里只使用 Z，因为只涉及 yaw 角度)
            rotation = R.from_euler('z', yaw)
            quaternion = rotation.as_quat()  # 返回四元数 [x, y, z, w]

            # 将目标位置相对于 base_link 的位置转换到 map 坐标系
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

            # 目标点的世界坐标 = base_link 的世界坐标 + 目标相对于 base_link 的坐标
            goal_pose.pose.position.x = transform.transform.translation.x + self.target_position['x'] - 0.11 * math.cos(yaw)
            goal_pose.pose.position.y = transform.transform.translation.y + self.target_position['y'] - 0.11 * math.sin(yaw)

            # 设置朝向 (四元数)
            goal_pose.pose.orientation.x = quaternion[0]
            goal_pose.pose.orientation.y = quaternion[1]
            goal_pose.pose.orientation.z = quaternion[2]
            goal_pose.pose.orientation.w = quaternion[3]

            # 开始导航
            self.get_logger().info(f"Navigating to goal: x={goal_pose.pose.position.x}, y={goal_pose.pose.position.y}")
            self.navigating = True
            self.navigator.goToPose(goal_pose)
            # 等待导航完成
            while not self.navigator.isTaskComplete():
                feedback = self.navigator.getFeedback()
                # self.get_logger().info(f"Current position: distance={feedback.distance_remaining}")
            self.get_logger().info(f"导航完成")
            self.publisher.publish(String(data=str(1)))
                
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform error: {e}")

def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
