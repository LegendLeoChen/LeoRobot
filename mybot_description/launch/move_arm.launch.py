from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from moveit_configs_utils.launch_utils import add_debuggable_node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("six_arm", package_name="mybot_description").to_moveit_configs()
    ld = LaunchDescription()

    # 声明 debug 参数并设置默认值为 false
    ld.add_action(DeclareLaunchArgument("debug", default_value="false", description="Enable debug mode"))

    # 声明其他参数
    ld.add_action(DeclareLaunchArgument("capabilities", default_value=""))
    ld.add_action(DeclareLaunchArgument("disable_capabilities", default_value=""))

    # 启动move_group
    my_generate_move_group_launch(ld, moveit_config)

    return ld

def my_generate_move_group_launch(ld, moveit_config):
    # 配置move_group节点参数
    move_group_configuration = {
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": True,
        "capabilities": LaunchConfiguration("capabilities"),
        "disable_capabilities": LaunchConfiguration("disable_capabilities"),
    }

    move_group_params = [
        moveit_config.to_dict(),
        move_group_configuration,
        {"use_sim_time": True}
    ]

    # 使用 LaunchConfiguration("debug") 来传递是否启用调试
    add_debuggable_node(
        ld,
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=move_group_params,
        extra_debug_args=["--debug"] if LaunchConfiguration("debug") == "true" else [],
    )
    return ld
