import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ClawController(Node):
    def __init__(self):
        super().__init__('claw_controller')
        self.publisher = self.create_publisher(JointTrajectory, '/gripper_controller/joint_trajectory', 10)
        self.timer = self.create_timer(1, self.send_joint_trajectory)           # 反复调用函数

    def send_joint_trajectory(self):
        # 创建JointTrajectory消息
        traj_msg = JointTrajectory()
        traj_msg.joint_names = ['joint7', 'joint8']

        # 设置关节目标位置
        point = JointTrajectoryPoint()
        point.positions = [-0.6, 0.6]  # 根据需要设置每个关节的目标位置
        point.time_from_start.sec = 1  # 在2秒内完成运动

        traj_msg.points.append(point)

        # 发布消息
        self.publisher.publish(traj_msg)
        self.get_logger().info('Publishing trajectory point')

def main(args=None):
    rclpy.init(args=args)
    node = ClawController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
