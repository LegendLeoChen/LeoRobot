import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import time
from std_msgs.msg import String

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        self.target_subscription = self.create_subscription(
            String,
            '/navigate_over',
            self.play,
            30
        )
        self.publisher = self.create_publisher(JointTrajectory, '/my_group_controller/joint_trajectory', 10)        # 机械臂动作组
        self.publisher2 = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)        # 机械爪动作组
        self.send_claw_trajectory([0.6, -0.6])
        time.sleep(1)
        self.send_arm_trajectory([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        # self.timer = self.create_timer(1, self.play)

    def play(self, msg):             # 夹取动作，张开爪子，弯曲机械臂，收紧爪子夹取，抬升机械臂
        self.send_claw_trajectory([0.6, -0.6])
        time.sleep(1)
        self.send_arm_trajectory([0.0, 0.0, 1.57, 0.0, 0.0, 0.0])
        time.sleep(2)
        self.send_claw_trajectory([-0.2, 0.2])
        time.sleep(2)
        self.send_arm_trajectory([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

    def send_arm_trajectory(self, target_point):        # 发布消息控制机械臂
        # 创建JointTrajectory消息
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # 设置关节目标位置
        point = JointTrajectoryPoint()
        point.positions = target_point  # 根据需要设置每个关节的目标位置
        point.time_from_start.sec = 1  # 在2秒内完成运动

        traj_msg.points.append(point)

        self.publisher.publish(traj_msg)
        # self.get_logger().info('Publishing trajectory point')
    
    def send_claw_trajectory(self, target_point):        # 发布消息控制机械爪
        # 创建JointTrajectory消息
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['joint7', 'joint8']

        # 设置关节目标位置
        point = JointTrajectoryPoint()
        point.positions = target_point  # 根据需要设置每个关节的目标位置
        point.time_from_start.sec = 1  # 在2秒内完成运动

        traj_msg.points.append(point)

        self.publisher2.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
