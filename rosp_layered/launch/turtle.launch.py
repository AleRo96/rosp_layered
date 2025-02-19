from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess



def generate_launch_description():
    
    turtle_node = Node(
        package= 'turtlesim',
        executable = 'turtlesim_node',
        name = 'turtlesim',
        output = "screen"

    )

    kill = ExecuteProcess(
        cmd= ['ros2','service', 'call', '/kill', 'turtlesim/srv/Kill', '{"name": "turtle1"}' ],
        output = 'screen'
    )

    spawn = ExecuteProcess(
        cmd= ['ros2','service', 'call', '/spawn', 'turtlesim/srv/Spawn', '{"x": 1.0, "y": 1.0, "name": "turtle"}' ],
        output = 'screen'
    )

    motion_node = Node(
        package= 'rosp_layered',
        executable = 'rosp_motion_control',
        name='rosp_motion_control',
        output = "screen"

    )

    return LaunchDescription([
        turtle_node,
        kill,
        spawn,
        motion_node,

    ])


