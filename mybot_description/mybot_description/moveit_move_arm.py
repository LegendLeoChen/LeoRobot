import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, RobotState, Constraints, PositionConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from scipy.spatial.transform import Rotation as R

class MoveitClient(Node):

    def __init__(self):
        super().__init__('moveit_move_arm_client')
        self.plan_client = self.create_client(GetMotionPlan, 'plan_kinematic_path')
        self.action_client = ActionClient(self, FollowJointTrajectory, '/my_group_controller/follow_joint_trajectory')
        while not self.plan_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.future = None
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10  # QoS参数，队列长度
        )
        self.joint_state = JointState()  # 存储最新的关节状态
        self.joint_state_received = False  # 用于标记关节状态是否已接收

    def send_request(self):
        request = GetMotionPlan.Request()

        # 创建规划请求
        request.motion_plan_request = MotionPlanRequest()
        request.motion_plan_request.group_name = 'my_group'  # 替换为你定义的group name
        request.motion_plan_request.allowed_planning_time = 2.0

        # 设置初始状态
        request.motion_plan_request.start_state = self.get_current_robot_state()

        # 设置目标位置和约束
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base_link'
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.
        target_pose.pose.position.z = 0.4

        r = R.from_euler('xyz', [0, 1.5708, 0])
        quat = r.as_quat()  # 返回的是 [x, y, z, w]
        # 设置目标姿态
        target_pose.pose.orientation.x = quat[0]
        target_pose.pose.orientation.y = quat[1]
        target_pose.pose.orientation.z = quat[2]
        target_pose.pose.orientation.w = quat[3]
        # 创建位置约束
        position_constraint = PositionConstraint()
        position_constraint.header = Header(frame_id='base_link')
        position_constraint.link_name = 'link6'
        
        # 定义约束区域
        position_constraint.constraint_region.primitives.append(SolidPrimitive())
        position_constraint.constraint_region.primitives[0].type = SolidPrimitive.BOX
        position_constraint.constraint_region.primitives[0].dimensions = [0.01, 0.01, 0.01]  # 假设这是一个0.1m的盒子区域
        position_constraint.constraint_region.primitive_poses.append(target_pose.pose)  # 添加位置约束的姿态
        position_constraint.weight = 1.0

        # 创建约束集合
        constraints = Constraints()
        constraints.position_constraints.append(position_constraint)

        request.motion_plan_request.goal_constraints.append(constraints)
        # 发送请求
        self.future = self.plan_client.call_async(request)

    def joint_state_callback(self, msg):
        """接收关节状态的回调函数"""
        self.joint_state = msg
        self.joint_state_received = True  # 收到关节状态后更新标记

    def get_current_robot_state(self):
        """获取当前机器人的状态"""
        current_state = RobotState()
        # 等待关节状态接收
        while not self.joint_state_received:
            self.get_logger().info('等待关节状态...')
            rclpy.spin_once(self)  # 处理ROS事件以接收消息

        current_state.joint_state = self.joint_state
        return current_state

    def process_response(self):
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            self.get_logger().info('运动规划成功')
            # 提取规划的轨迹
            # print(self.future.result())
            planned_trajectory = self.future.result().motion_plan_response.trajectory
            self.execute_trajectory(planned_trajectory)  # 执行轨迹
        else:
            self.get_logger().error('运动规划失败')

    def execute_trajectory(self, trajectory):
        """执行规划的轨迹"""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Action server not available!')
            return
        goal_msg = FollowJointTrajectory.Goal()
        # 将 MoveIt 生成的 trajectory 转换为 JointTrajectory
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = trajectory.joint_trajectory.joint_names
        for point in trajectory.joint_trajectory.points:
            trajectory_point = JointTrajectoryPoint()
            trajectory_point.positions = point.positions
            trajectory_point.velocities = point.velocities
            trajectory_point.accelerations = point.accelerations
            trajectory_point.time_from_start = point.time_from_start
            joint_trajectory.points.append(trajectory_point)
        
        # 设置目标
        goal_msg.trajectory = joint_trajectory

        # 发送 action goal
        self.action_client.send_goal_async(goal_msg)
        self.get_logger().info('轨迹已发送，等待机械臂执行...')


def main(args=None):
    rclpy.init(args=args)
    motion_plan_client = MoveitClient()
    motion_plan_client.send_request()
    motion_plan_client.process_response()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
