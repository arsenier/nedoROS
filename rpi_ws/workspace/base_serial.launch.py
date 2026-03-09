from launch import LaunchDescription
from launch import actions
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    zmrobo_driver = Node(
            package='zmrobo_init',
            executable='zmrobo_node',
            name='zmrobo',
            output='screen',
            parameters=[{
                'usart_port_name': '/dev/ttyACM1',
                'serial_baud_rate': 115200,
                'robot_frame_id': 'base_footprint',
                'akm_cmd_vel': 'none',
                'product_number': 0,
                'cmd_vel': 'cmd_vel', 
            }],
            # remappings=[
            #     (LaunchConfiguration('cmd_vel'), 'smoother_cmd_vel')
            # ] if LaunchConfiguration('smoother') else [
            #     (LaunchConfiguration('cmd_vel'), 'cmd_vel')
            # ]
    )

    return LaunchDescription([
        zmrobo_driver
    ])
    # return LaunchDescription([
    #     # Define arguments
    #     actions.DeclareLaunchArgument('akm', default_value='false'),
    #     actions.DeclareLaunchArgument('smoother', default_value='false'),
    #     actions.DeclareLaunchArgument('akm_cmd_vel_topic', 
    #                                    default_value=['/ackermann_cmd', IfCondition(LaunchConfiguration('akm'))])
        
    #     # First node
    #     Node(
    #         package='zmrobo_init',
    #         executable='zmrobo_node',
    #         name='zmrobo',
    #         output='screen',
    #         parameters=[{
    #             'usart_port_name': '/dev/ttyACM1',
    #             'serial_baud_rate': 115200,
    #             'robot_frame_id': 'base_footprint',
    #             'akm_cmd_vel': LaunchConfiguration('akm_cmd_vel_topic'),
    #             'product_number': 0,
    #             'cmd_vel': IfCondition(LaunchConfiguration('smoother')), 
    #         }],
    #         remappings=[
    #             (LaunchConfiguration('cmd_vel'), 'smoother_cmd_vel')
    #         ] if LaunchConfiguration('smoother') else [
    #             (LaunchConfiguration('cmd_vel'), 'cmd_vel')
    #         ]
    #     ),

    #     # Second node
    #     Node(
    #         package='turn_on_wheeltec_robot',
    #         executable='cmd_vel_to_ackermann_drive.py',
    #         name='cmd2ackermann',
    #         output='screen',
    #         condition=IfCondition(LaunchConfiguration('akm')),
    #         remappings=[
    #             (LaunchConfiguration('cmd_vel'), 'smoother_cmd_vel') 
    #         ] if LaunchConfiguration('smoother') else []
    #     ),

    #     # Include smoother launch file if required
    #     actions.IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([FindPackageShare('zmrobo_init'), '/launch/include/velocity_smoother.launch.py']),
    #         condition=IfCondition(LaunchConfiguration('smoother')),
    #     ),
    # ])
