#!/usr/bin/env python3
"""
VTOL TAKTİK İHA — Yer İstasyonu Başlatıcı (GCS Orkestra Şefi)
================================================================
Bu dosya sahadaki operatörün bilgisayarında çalışır.
Komut paneli (GCS Panel) ve arka plan dinleyicisini (GCS Listener)
tek seferde ayağa kaldırır.

⚠ DİKKAT: Bu dosya uçağın uçuş dinamiğine veya yapay zekasına
ASLA karışmaz. Sadece operatör arayüzü ve telemetri dinleyicisini
içerir.

Kullanım:
    ros2 launch vtol_bringup gcs.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ╔══════════════════════════════════════════════════════════╗
    # ║  1. GCS PANEL — Operatör Komut Terminali                ║
    # ║     Operatörün hedef koordinat gönderdiği, ENGAGE/RTL   ║
    # ║     komutları verdiği yeşil terminal arayüzü.           ║
    # ║     Klavye girdisi alabilmesi için emulate_tty=True.    ║
    # ╚══════════════════════════════════════════════════════════╝
    gcs_panel_node = Node(
        package='vtol_control',
        executable='gcs_panel',
        name='gcs_panel',
        output='screen',
        emulate_tty=True,
        # prefix ile ayrı bir terminal penceresi açılabilir (opsiyonel):
        # prefix='xterm -e',
    )

    # ╔══════════════════════════════════════════════════════════╗
    # ║  2. GCS LISTENER — Arka Plan Telemetri Dinleyicisi      ║
    # ║     Uçaktan gelen durum güncellemelerini, mission        ║
    # ║     loglarını ve uyarıları dinleyip terminale basar.     ║
    # ╚══════════════════════════════════════════════════════════╝
    gcs_listener_node = Node(
        package='vtol_control',
        executable='gcs_listener',
        name='gcs_listener',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        gcs_panel_node,
        gcs_listener_node,
    ])
