#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bringup_share = get_package_share_directory('vtol_bringup')

    # 1. MicroXRCEAgent - uXRCE-DDS Bridge (Default PX4 port 8888)
    # respawn=True: Agent çökerse otomatik yeniden başlar
    xrce_dds_process = ExecuteProcess(
        cmd=[
            'MicroXRCEAgent', 'udp4', '-p', '8888'
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
    )

    # 2. UAV Sinir Sistemi (Nested Launch)
    # GÖZDEN KAÇIRMA: uav.launch.py içindeki stream_node'a 
    # 'use_sim_time': True parametresini uav.launch içinden vermen gerekebilir.
    uav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'uav.launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items(), # Simülasyon zamanını aktif et
    )

    # uXRCE-DDS Agent bağlantısı için 5 saniye bekle
    delayed_uav_launch = TimerAction(
        period=5.0,
        actions=[uav_launch],
    )

    return LaunchDescription([
        xrce_dds_process,
        # gz_camera_bridge KALDIRILDI! Classic'te köprü kullanılmaz.
        delayed_uav_launch,
    ])