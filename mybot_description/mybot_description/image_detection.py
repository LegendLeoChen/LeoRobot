import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import torch
import os
import numpy as np
from ultralytics import YOLO
import tf2_ros
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
import re

class ImageDisplayNode(Node):

    def __init__(self):
        super().__init__('image_display_node')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',                # 订阅的rgb图像的话题
            self.listener_callback,
            10
        )
        self.depth_subscription = self.create_subscription(
            Image,
            '/depth_camera/depth/image_raw',    # 订阅深度图像的话题
            self.depth_listener_callback,
            10
        )
        self.target_subscription = self.create_subscription(
            String,
            '/voice_commands',                  # 订阅语言节点的指令
            self.target_callback,
            10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',          # 订阅相机参数
            self.camera_info_callback,
            10
        )
        self.nav_subscription = self.create_subscription(
            String,
            '/vision_on',                  # 订阅导航节点更新任务索引
            self.nav_callback,
            10
        )
        self.publisher_nav = self.create_publisher(Float32MultiArray, 'target_position', 30)        # 创建话题，告知导航节点目标位置
        self.publisher_wheel = self.create_publisher(String, 'wheel_control', 10)        # 创建话题，控制差速节点进行位姿调节
        # 初始化 CV Bridge
        self.br = CvBridge()

        # 加载 YOLOv8 模型
        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, '../../../../share/mybot_description/weights/yolov8l.pt')
        self.model = YOLO(model_path)
        self.depth_image = None
        self.target = -1

        # 初始化相机内参
        self.camera_fx = None
        self.camera_fy = None
        self.camera_cx = None
        self.camera_cy = None

        # 初始化TF变换监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.command = ''               # 语言节点的指令
        self.task = -1                  # 当前任务索引
        self.classes = []               # 类别列表
        self.correcting = False         # 是否在校准位姿
        self.send_target = False        # 是否已经发送过位置，保证发送一次就行

    def camera_info_callback(self, msg):
        """获取相机的内参信息"""
        self.camera_fx = msg.k[0]       # 焦距 fx
        self.camera_fy = msg.k[4]       # 焦距 fy
        self.camera_cx = msg.k[2]       # 光心 cx
        self.camera_cy = msg.k[5]       # 光心 cy
        # self.get_logger().info(f"Camera info received: fx={self.camera_fx}, fy={self.camera_fy}, cx={self.camera_cx}, cy={self.camera_cy}")

    def target_callback(self, msg):
        if self.command:                # 有目标就不再接收了
            return
        self.command = msg.data
        if msg.data[0].isupper():       # 判断有无区域信息
            self.classes = re.findall(r'\d+', msg.data) 
        else:                           # 无区域则直接开找
            self.target = int(msg.data)
    
    def nav_callback(self, msg):
        if msg.data == 'correct':           # 校准操作，启动小车令其转动
            self.correcting = True
            self.publisher_wheel.publish(String(data=str(1)))       # 启动小车自转
        else:                               # 普通的寻找目标操作
            self.task = int(msg.data)
            self.target = int(self.classes[self.task])       # 更新当前的任务
            self.get_logger().info(f"任务已更新: {self.task} ，类别：{self.model.names[self.target]}-{self.target}")

    def listener_callback(self, msg):
        # 检查是否已经接收到相机内参
        if self.camera_fx is None or self.camera_fy is None:
            self.get_logger().error('Camera intrinsics not yet received.')
            return
        # 将 ROS 图像消息转换为 OpenCV 图像
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # 进行推理
        results = self.model(cv_image, verbose=False)
        # 绘制检测结果
        for i, result in enumerate(results):
            boxes = result.boxes
            if i != 0:
                annotated_frame = annotated_frame.plot()        # 绘制预测框
            else:
                annotated_frame = result.plot()
            for box in boxes:                                   # 获得深度（目标距离）并绘制
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                depth_value = self.get_depth_value(x1, y1, x2, y2)
                # self.get_logger().info(f"Detected object depth: {depth_value:.2f} meters")
                if int(box.cls) == self.target and not np.isnan(depth_value):
                    if not self.correcting and not self.send_target:        # 排除校准位姿或已经发送过的情况
                        # print(f'目标的边界框位置 {self.model.names[self.target]}：{x1}, {y1}, {x2}, {y2}')
                        coordinates = self.get_coordinates((x1 + x2) // 2, (y1 + y2) // 2, depth_value)
                        if coordinates:
                            print(f"目标{self.model.names[self.target]}的相对坐标: {coordinates}")
                            msg = Float32MultiArray()
                            msg.data = [self.task * 1.0] + list(coordinates)
                            self.publisher_nav.publish(msg)
                            self.send_target = True             # 以及发送就不用再发送了
                    elif self.correcting:                   # 校准位姿
                        if abs((x1 + x2) // 2 - self.camera_cx) < 15 and depth_value < 1.0 and depth_value > 0.0:
                            self.publisher_wheel.publish(String(data=str(2)))                 # 距离偏远，小车前进
                        elif abs((x1 + x2) // 2 - self.camera_cx) < 15 and depth_value < 0.0:        # 角度已经对准目标
                            self.publisher_wheel.publish(String(data=str(0)))                 # 停止小车
                            self.publisher_nav.publish(Float32MultiArray(data=[1.0]))         # 告知导航节点可以抓取
                            self.correcting = False
                            self.send_target = False
                            self.target = -1
                        
                cv2.putText(annotated_frame, f"{depth_value:.2f}m", 
                            (x1, y1 - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 0, 0), 4)
        # 显示图像
        cv2.imshow("YOLOv8 Detection", annotated_frame)
        cv2.waitKey(1)  # 等待1毫秒，以便窗口更新

    def depth_listener_callback(self, msg):
        # 将 ROS 深度图像消息转换为 OpenCV 格式的深度图像
        self.depth_image = self.br.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def get_depth_value(self, x1, y1, x2, y2):
        """获取检测框中心像素的深度值"""
        if self.depth_image is not None:
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2
            # 从深度图像中获取中心像素的深度值
            depth_value = self.depth_image[center_y, center_x]
            if np.isnan(depth_value) or depth_value == 0:
                return float('nan')     # 无效深度值
            return depth_value          # 深度值以米为单位

        return float('nan')
    
    def get_coordinates(self, u, v, depth):
        """将像素坐标和深度值转换为base_link坐标"""
        # 构造相机坐标系下的点
        camera_point = PointStamped()
        camera_point.header.frame_id = 'camera_link'
        camera_point.point.y = - (u - self.camera_cx) * depth / self.camera_fx
        camera_point.point.z = - (v - self.camera_cy) * depth / self.camera_fy
        camera_point.point.x = float(depth)
        # print(camera_point)
        try:
            # 等待 TF 可用
            self.tf_buffer.can_transform('base_link', 'camera_link', rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0))
            # 执行转换
            point_base_link = self.tf_buffer.transform(camera_point, 'base_link', timeout=rclpy.duration.Duration(seconds=1.0))
            # 返回转换后的坐标
            return (point_base_link.point.x, point_base_link.point.y, point_base_link.point.z)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().error(f"Transform failed: {e}")
            return None

def main(args=None):
    rclpy.init(args=args)
    image_display_node = ImageDisplayNode()
    rclpy.spin(image_display_node)
    image_display_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()