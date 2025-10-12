# 导入所需的库
from launch import LaunchDescription
from launch_ros.actions import Node

# 所有launch文件都必须包含一个名为 generate_launch_description 的函数
def generate_launch_description():
    
    # 创建一个LaunchDescription对象
    ld = LaunchDescription()

    # 定义要启动的节点
    detect_and_solve = Node(
        package='detect_and_solve',
        executable='detect_and_solve',
        # name='my_talker_1'  # 可选：为节点设置一个自定义名称
    )

    # listener_node = Node(
    #     package='my_first_sub',
    #     executable='subscriber_node',
    #     name='my_listener_1'
    # )

    # 将节点添加到LaunchDescription中
    ld.add_action(detect_and_solve)
    # ld.add_action(listener_node)

    # 返回LaunchDescription对象
    return ld