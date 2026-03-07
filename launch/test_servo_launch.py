import launch
import launch_ros.actions
from launch.actions import TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_launch_description():
    # 1. 定义 Launch 参数
    servo_type_arg = LaunchConfiguration('servo_type', default='rs485')
    
    # RS485 舵机参数
    servo_port_arg = LaunchConfiguration('port', default='/dev/ttyUSB1')
    servo_id_arg = LaunchConfiguration('servo_id', default='1')
    
    # PWM 舵机参数
    pwm_period_arg = LaunchConfiguration('pwm_period', default='10000')
    pwm_duty_min_arg = LaunchConfiguration('pwm_duty_min', default='250')
    pwm_duty_max_arg = LaunchConfiguration('pwm_duty_max', default='1250')
    pwm_reg_mux_arg = LaunchConfiguration('pwm_reg_mux', default='0x00C40000D0')
    pwm_reg_period_arg = LaunchConfiguration('pwm_reg_period', default='0x00C408002C')
    pwm_reg_low_arg = LaunchConfiguration('pwm_reg_low', default='0x00C4080030')
    pwm_reg_high_arg = LaunchConfiguration('pwm_reg_high', default='0x00C4080034')
    
    # 2. 启动 Service 提供者节点 (servo_node)
    servo_node = launch_ros.actions.Node(
        package='servo_rs485_ros2',
        executable='servo_node',
        name='servo_rs485_node',
        output='screen',
        parameters=[
            # 通用参数
            {'servo_type': servo_type_arg},
            {'publish_interval_ms': 30},
            # RS485 参数
            {'port': servo_port_arg},
            {'servo_id': servo_id_arg},
            {'baudrate': 115200},
            # PWM 参数
            {'pwm_period': pwm_period_arg},
            {'pwm_duty_min': pwm_duty_min_arg},
            {'pwm_duty_max': pwm_duty_max_arg},
            {'pwm_reg_mux': pwm_reg_mux_arg},
            {'pwm_reg_period': pwm_reg_period_arg},
            {'pwm_reg_low': pwm_reg_low_arg},
            {'pwm_reg_high': pwm_reg_high_arg},
        ]
    )

    # 3. 定义 get_angle 服务定时调用
    get_angle_call = launch.actions.ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/get_angle', 'servo_rs485_ros2/srv/GetAngle', '{}'],
        output='screen'
    )

    # 4. 100Hz定时调用 get_angle 服务
    get_angle_timer = TimerAction(
        period=0.01,  # 100Hz
        actions=[get_angle_call]
    )

    # 5. 其他服务只在启动时调用一次
    def call_once_services(context):
        ping_servo_call = launch.actions.ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/ping_servo', 'servo_rs485_ros2/srv/PingServo', '{}'],
            output='screen'
        )
        set_angle_call = launch.actions.ExecuteProcess(
            cmd=['ros2', 'service', 'call', '/set_angle', 'servo_rs485_ros2/srv/SetAngle', TextSubstitution(text='{degree: 8.0, time_ms: 500}')],
            output='screen'
        )
        return [ping_servo_call, set_angle_call]

    once_timer = TimerAction(
        period=5.0,
        actions=[OpaqueFunction(function=call_once_services)]
    )

    return launch.LaunchDescription([
        # 通用参数
        launch.actions.DeclareLaunchArgument(
            'servo_type',
            default_value='rs485',
            description='Servo type: "rs485" or "pwm"'
        ),
        # RS485 参数
        launch.actions.DeclareLaunchArgument(
            'port',
            default_value='/dev/ttyUSB1',
            description='RS485 Serial Port'
        ),
        launch.actions.DeclareLaunchArgument(
            'servo_id',
            default_value='1',
            description='RS485 Servo ID'
        ),
        # PWM 参数
        launch.actions.DeclareLaunchArgument(
            'pwm_period',
            default_value='10000',
            description='PWM period register value'
        ),
        launch.actions.DeclareLaunchArgument(
            'pwm_duty_min',
            default_value='250',
            description='PWM duty cycle min (for -135 deg)'
        ),
        launch.actions.DeclareLaunchArgument(
            'pwm_duty_max',
            default_value='1250',
            description='PWM duty cycle max (for +135 deg)'
        ),
        launch.actions.DeclareLaunchArgument(
            'pwm_reg_mux',
            default_value='0x00C40000D0',
            description='PWM pin mux register address'
        ),
        launch.actions.DeclareLaunchArgument(
            'pwm_reg_period',
            default_value='0x00C408002C',
            description='PWM period register address'
        ),
        launch.actions.DeclareLaunchArgument(
            'pwm_reg_low',
            default_value='0x00C4080030',
            description='PWM low start register address'
        ),
        launch.actions.DeclareLaunchArgument(
            'pwm_reg_high',
            default_value='0x00C4080034',
            description='PWM high start (duty) register address'
        ),
        servo_node,
        once_timer,
        get_angle_timer,
    ])