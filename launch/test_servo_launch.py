import launch
import launch_ros.actions
from launch.actions import TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # 1. 定义 Launch 参数
    servo_port_arg = LaunchConfiguration('port', default='/dev/ttyUSB1')
    servo_id_arg = LaunchConfiguration('servo_id', default='1')
    
    # 2. 启动 Service 提供者节点 (servo_node)
    servo_node = launch_ros.actions.Node(
        package='servo_rs485_ros2',
        executable='servo_node',
        name='servo_rs485_node',
        output='screen',
        parameters=[
            {'port': servo_port_arg},
            {'servo_id': servo_id_arg},
            {'baudrate': 115200},
        ]
    )

    # 3. 定义 Service 调用函数 (使用 OpaqueFunction 保证在运行时执行)
    def call_service_commands(context):
        # 确保在启动命令中替换了正确的 Service 类型
        ping_servo_call = launch.actions.ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/ping_servo', 'servo_rs485_ros2/srv/PingServo', '{}'],
            output='screen'
        )
        
        set_angle_call = launch.actions.ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/set_angle', 'servo_rs485_ros2/srv/SetAngle', TextSubstitution(text='{degree: 5.0, time_ms: 500}')],
            output='screen'
        )
        
        get_angle_call = launch.actions.ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/get_angle', 'servo_rs485_ros2/srv/GetAngle', '{}'],
            output='screen'
        )
        
        return [ping_servo_call, set_angle_call, get_angle_call]

    # 4. 延迟调用 Service (给 servo_node 启动时间)
    service_calls_timer = TimerAction(
        period=5.0,  # 延迟 5 秒
        actions=[
            OpaqueFunction(function=call_service_commands)
        ]
    )

    return launch.LaunchDescription([
        # 允许用户通过命令行覆盖参数
        launch.actions.DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB1',
            description='RS485 Serial Port'
        ),
        launch.actions.DeclareLaunchArgument(
            'servo_id',
            default_value='1',
            description='Default Servo ID'
        ),
        servo_node,
        service_calls_timer,
    ])