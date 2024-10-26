import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from copy import deepcopy
import tf2_ros
from geometry_msgs.msg import PointStamped
import math
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from std_msgs.msg import String
import re
import tf2_geometry_msgs 
import time

# 预设动作阶段
TO_AREA = 0
GET_AREA = 1
TO_TARGET = 2
GET_TARGET = 3
GET_CORRECT = 4
GRASP = 5
TO_ORIGIN = 6
GET_ORIGIN = 7
WAITING = -1

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

        self.target_subscription = self.create_subscription(
            Float32MultiArray,
            '/target_position',                 # 订阅视觉节点的目标位置
            self.target_callback,
            30
        )
        self.target_subscription = self.create_subscription(
            String,
            '/voice_commands',                  # 订阅语言节点的指令
            self.command_callback,
            10
        )
        self.grasp_subscription = self.create_subscription(
            String,
            '/action_over',                     # 订阅抓取节点的指令
            self.action_callback,
            10
        )
        self.publisher_grasp = self.create_publisher(String, 'navigate_over', 10)        # 创建话题，告知抓取节点导航结束开始夹取
        self.publisher_vision = self.create_publisher(String, 'vision_on', 10)        # 创建话题，告知视觉节点下一个任务索引

        # TF 变换监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.target_position = None     # 用于存储目标位置的变量
        self.navigating = False         # 标志位，表示当前是否在导航
        self.state = WAITING            # 当前动作阶段
        self.command = ''               # 语言节点指令
        self.task = -1                  # 当前任务索引
        self.areas = []                 # 指令解析出的区域
        self.classes = []               # 指令解析出的类别
        self.area_dict = {'A': {'x': 5.0, 'y': 2.0}, 'B': {'x': -3.0, 'y': -2.5}, 'O': {'x': 0.01, 'y': 0.01}}      # 区域字典

    def target_callback(self, msg):
        """目标位置订阅回调"""
        if len(msg.data) == 1 and msg.data[0] == 1.0 and self.state == GET_TARGET:        # 校准位姿完成
            self.state = GET_CORRECT
            self.get_logger().info(f"当前状态: 已校准位姿")
        elif len(msg.data) == 4:        # 有区域的目标位置信息
            if int(msg.data[0]) == self.task and self.state == GET_AREA:
                self.target_position = {
                    'x': msg.data[1],
                    'y': msg.data[2],
                    'z': msg.data[3]
                }
                self.state = TO_TARGET
                self.get_logger().info(f"当前状态: 导航前往目标航点")
        elif len(msg.data) == 3:        # 无区域的目标信息
            self.allow = True
            self.target_position = {
                'x': msg.data[0],
                'y': msg.data[1],
                'z': msg.data[2]
            }
    
    def command_callback(self, msg):
        """语言节点的目标位置处理"""
        if self.command:                # 有命令就不处理了
            return
        if len(msg.data) > 0 and msg.data[0].isupper():
            self.state = TO_AREA
            self.command = msg.data
            self.task = 0                                        # 第一个任务
            self.areas = re.findall(r'[A-Za-z]+', msg.data)      # 区域
            self.classes = re.findall(r'\d+', msg.data)          # 类别
            self.get_logger().info(f"接收到语言节点的指令: {self.command}")
            self.get_logger().info(f"当前状态: 导航前往观察航点")
            self.timer = self.create_timer(2, self.navigate_play)       # 反复调用导航函数

    def action_callback(self, msg):
        if self.state == GRASP:             # 已抓到目标，回原点
            self.state = TO_ORIGIN
            self.get_logger().info(f"当前状态: 导航前往原点")
        elif self.state == GET_ORIGIN:      # 已放开目标，去下一个区域的观察点
            self.state = TO_AREA
            self.get_logger().info(f"当前状态: 导航前往观察航点")

    def navigate_play(self):
        if self.navigating == True or self.command == '' or self.task >= len(self.areas):
            return
        if self.state == TO_AREA:           # 前往观察航点，不tf，到达后告知视觉节点
            fb = self.navigate_to_target(self.area_dict[self.areas[self.task]])
            if fb:
                self.state = GET_AREA
                self.get_logger().info(f"当前状态: 到达观察航点")
                time.sleep(2)
                self.publisher_vision.publish(String(data=str(self.task)))
        elif self.state == TO_TARGET:       # 前往目标航点，执行tf，到达后告知视觉节点
            fb = self.navigate_to_target(self.target_position, tf=True)
            if fb:
                self.state = GET_TARGET
                self.get_logger().info(f"当前状态: 到达目标航点，开始位姿校准")
                self.publisher_vision.publish(String(data='correct'))
        elif self.state == GET_CORRECT:       # 夹取
            self.get_logger().info(f"当前状态: 夹取目标")
            self.state = GRASP
            self.publisher_grasp.publish(String(data=str(1)))
        elif self.state == TO_ORIGIN:       # 前往原点，不tf，到达后放开爪子
            fb = self.navigate_to_target(self.area_dict['O'],)
            if fb:
                self.state = GET_ORIGIN
                self.get_logger().info(f"当前状态: 到达原点，放开目标")
                self.publisher_grasp.publish(String(data=str(2)))
                self.task += 1              # 开始下一步任务
                if self.task >= len(self.areas):
                    self.get_logger().info(f"已完成所有任务")
                else:
                    self.get_logger().info(f"开始执行任务{self.task}")

    def navigate(self, point):
        """导航到某点"""
        self.navigator.goToPose(point)
        # 等待导航完成
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            # self.get_logger().info(f"Current position: distance={feedback.distance_remaining}")
        self.get_logger().info(f"导航完成")

    def navigate_to_target(self, point, tf=False):       # tf为True需要转换
        """执行导航到目标点"""
        # 获取 base_link 相对于 map 坐标系的变换
        try:
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            except:
                return False
            self.get_logger().info(f"当前机器人位置: {transform.transform.translation.x}, {transform.transform.translation.y}, {transform.transform.translation.z}")
            self.navigating = True

            if tf:      # 目标相对于base_link的坐标->目标的世界坐标
                point = self.transform_to_map(point, transform)
            # 计算目标点相对于机器人的 yaw 角度（单位：弧度）
            yaw = math.atan2(point['y'] - transform.transform.translation.y, point['x'] - transform.transform.translation.x)

            # 使用 scipy 将 yaw 转换为四元数 (欧拉角顺序为 ZYX, 这里只使用 Z，因为只涉及 yaw 角度)
            rotation = R.from_euler('z', yaw)
            quaternion = rotation.as_quat()  # 返回四元数 [x, y, z, w]

            # 将目标位置相对于 base_link 的位置转换到 map 坐标系
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

            if tf:      # 是目标航点，目标点的世界坐标，0.5保持距离
                self.get_logger().info(f"目标在{point['x']}, {point['y']}")
                goal_pose.pose.position.x = point['x'] - 0.5 * math.cos(yaw)
                goal_pose.pose.position.y = point['y'] - 0.5 * math.sin(yaw)
            else:       # 是观察航点或原点，世界坐标
                goal_pose.pose.position.x = point['x']
                goal_pose.pose.position.y = point['y']
            # 设置朝向 (四元数)，回原点不需要
            goal_pose.pose.orientation.x = quaternion[0]
            goal_pose.pose.orientation.y = quaternion[1]
            goal_pose.pose.orientation.z = quaternion[2]
            goal_pose.pose.orientation.w = quaternion[3]
            # 导航开始
            self.navigate(goal_pose)
            self.navigating = False
            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform error: {e}")
    
    def transform_to_map(self, point, transform):
        '''将base_link坐标系坐标转为世界坐标（map坐标系）'''
        base_point = PointStamped()
        base_point.header.frame_id = 'base_link'
        base_point.point.x = point['x']
        base_point.point.y = point['y']
        base_point.point.z = point['z']

        transformed_point = tf2_geometry_msgs.do_transform_point(base_point, transform)
        return {'x': transformed_point.point.x, 'y': transformed_point.point.y, 'z': transformed_point.point.z}


def main(args=None):
    rclpy.init(args=args)
    navigation_node = NavigationNode()
    rclpy.spin(navigation_node)
    navigation_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
