import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class VelocityController(Node):
    def __init__(self):
        super().__init__('velocity_controller')
        self.target_subscription = self.create_subscription(
            String,
            '/wheel_control',                  # 订阅语言节点的指令
            self.command_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 每0.1秒调用一次
        self.vel_msg = Twist()

        # 设置初始速度
        self.set_velocity(0.0, 0.0)  # 线速度和角速度

    def command_callback(self, msg):
        if msg.data == '0':         # 停止
            self.set_velocity(0.0, 0.0)
            self.get_logger().info(f"停止运动")
        elif msg.data == '1':       # 转动
            self.set_velocity(0.0, 0.2)
            self.get_logger().info(f"开始校准角度")
        elif msg.data == '2':       # 移动
            self.set_velocity(0.02, 0.0)
            self.get_logger().info(f"开始校准距离")

    def set_velocity(self, linear, angular):
        """设置线速度和角速度"""
        self.vel_msg.linear.x = linear
        self.vel_msg.angular.z = angular

    def timer_callback(self):
        """发布速度消息"""
        self.publisher_.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = VelocityController()
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
