import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    
    pkg_sim = get_package_share_directory('irc_simulation')
    world_path = os.path.join(pkg_sim, 'worlds', 'irc.world')

    
    model_path = os.path.join(pkg_sim, 'models')
    rviz_config_path = os.path.join(pkg_sim, 'rviz', 'camera_view.rviz')
    bridge_yaml_path = os.path.join(pkg_sim, 'config', 'rviz.yaml')
    resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    # 1. モデルへのパスを通す
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', 
        value=f'{model_path}:{resource_path}'
    )

    # 2. Gazebo (gz_sim) を起動
    # (これが構文エラーになっていた箇所です)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            # '-r' (Run) フラグで最初から再生状態にする
            'gz_args': f'-r {world_path}',
        }.items(),
    )

    # 3. カメラ映像のブリッジを起動 (元からOK)
    bridge_params = os.path.join(get_package_share_directory("irc_simulation"),'config', 'rviz.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )
    
    # 4. TF (座標変換) のブリッジを起動 (修正済み)
    tf_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='tf_bridge',
        arguments=[
            # 正しいTFトピック名に変更
            '/world/irc_world/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        output='screen'
    )

    # 5. RViz2 を起動 (元からOK)
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        set_model_path, 
        gz_sim,
        ros_gz_bridge,
        tf_bridge,
        rviz
    ])
