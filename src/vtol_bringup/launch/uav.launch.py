#!/usr/bin/env python3
"""
VTOL TAKTİK İHA — Ana Başlatıcı (Orkestra Şefi)
==================================================
Bu dosya tüm sistemi ayağa kaldıran merkezi başlatıcıdır.
YAML konfigürasyon dosyalarını okur ve içindeki güncel değerleri
ilgili node'lara parametre olarak enjekte eder.

Kullanım:
    ros2 launch vtol_bringup uav.launch.py

Sahada değer değiştirmek için:
    1. config/mission_params.yaml  → Uçuş & pilotaj ayarları
    2. config/camera_params.yaml   → Kamera & yapay zeka ayarları
    dosyalarını düzenleyip sistemi yeniden başlatın. Derleme gerekmez.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Konfigürasyon Dosya Yollarını Bul ─────────────────────
    bringup_share = get_package_share_directory('vtol_bringup')

    mission_params_path = os.path.join(bringup_share, 'config', 'mission_params.yaml')
    camera_params_path  = os.path.join(bringup_share, 'config', 'camera_params.yaml')

    # ╔══════════════════════════════════════════════════════════╗
    # ║  NODE TANIMLARI                                         ║
    # ║  Her node, ilgili YAML dosyasından parametrelerini alır ║
    # ╚══════════════════════════════════════════════════════════╝

    from launch.actions import DeclareLaunchArgument
    from launch.substitutions import LaunchConfiguration

    use_sim_time = LaunchConfiguration('use_sim_time')

    sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. GÖREV YÖNETİCİSİ (Beyni / Komutan)
    mission_manager_node = Node(
        package='vtol_control',
        executable='state_machine',
        name='vtol_mission_manager',
        output='screen',
        parameters=[mission_params_path, {'use_sim_time': use_sim_time}],
        emulate_tty=True,
    )

    # 2. AVCI PİLOT (Görsel Servo / Kamikaze Dalışçı)
    visual_servo_node = Node(
        package='vtol_control',
        executable='visual_servo',
        name='visual_servo_node',
        output='screen',
        parameters=[mission_params_path, {'use_sim_time': use_sim_time}],
        emulate_tty=True,
    )

    # 3. YAPAY ZEKA GÖZÜ — ÖN KAMERA (Vision / Hedef Tanıma)
    vision_node = Node(
        package='vtol_vision',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[camera_params_path, {'use_sim_time': use_sim_time}],
        emulate_tty=True,
    )

    # 3b. YAPAY ZEKA GÖZÜ — ALT KAMERA
    down_vision_node = Node(
        package='vtol_vision',
        executable='vision_node',
        name='down_vision_node',
        output='screen',
        parameters=[camera_params_path, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/vision/target_error', '/down_vision/target_error'),
            ('/vision/target_state', '/down_vision/target_state'),
            ('/vision/debug_image',  '/down_vision/debug_image'),
        ],
        emulate_tty=True,
    )

    # 4. KAMERA YAYIN MODÜLÜ (Uçak Üstü Kamera)
    camera_stream_node = Node(
        package='vtol_vision',
        executable='stream_node',
        name='camera_stream_node',
        output='screen',
        parameters=[camera_params_path, {'use_sim_time': use_sim_time}],
        emulate_tty=True,
    )

    return LaunchDescription([
        sim_arg,
        mission_manager_node,
        visual_servo_node,
        vision_node,
        down_vision_node,
        camera_stream_node,
    ])
