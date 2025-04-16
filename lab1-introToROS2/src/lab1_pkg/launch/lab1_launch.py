import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        #Declarar el parámetro del talker 
        DeclareLaunchArgument(
            'talker_rate',
            default_value='1.0',
            description='Rate at wich the talker node will publish messages'
        ),

        #Incluir o ejecutar el nodo talker
        Node(
            package='lab1_pkg',
            executable='talker',
            name='talker',
            output='screen',
            parameters=[{'rate': LaunchConfiguration('talker_rate')}]
        ),

        # Incluir o ejecutar el nodo relay
        Node(
            package='lab1_pkg',
            executable='relay',
            name='relay',
            output='screen'
        ),
    ])