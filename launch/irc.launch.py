import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    
    pkg_sim = get_package_share_directory('irc_simulation')
    world_path = os.path.join(pkg_sim, 'worlds', 'irc.world')
    
    # 'models' ディレクトリへのパス
    model_path = os.path.join(pkg_sim, 'models')

    # 既存のパスを取得 (もしあれば)
    resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')

    # ⇐ ここを変更
    # Ignition Fortress (Gazebo Sim) がモデルを見つけるための環境変数を設定
    set_model_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH', # ⇐ 'GAZEBO_MODEL_PATH' から変更
        value=f'{model_path}:{resource_path}' # ⇐ 'model_path' を既存のパスに追加
    )

    # 'gz_sim.launch.py' (Ignition Fortress) を起動する設定
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': world_path,
        }.items(),
    )

    return LaunchDescription([
        set_model_path, # ⇐ 変更した変数を適用
        gz_sim
    ])
