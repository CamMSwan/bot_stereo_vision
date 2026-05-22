#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Caminhos do Pacote e Configurações
    pkg_vision = get_package_share_directory('bot_stereo_vision')
    config_front = os.path.join(pkg_vision, 'config', 'front_camera.yaml')
    config_back = os.path.join(pkg_vision, 'config', 'back_camera.yaml')

    nodes = []

    # --- 1. NÓ DE VISÃO ESTÉREO (ORQUESTRADOR MULTI-CÂMERA) ---
    # Agora este nó gerencia Front e Back simultaneamente
    stereo_node = Node(
        package='bot_stereo_vision',
        executable='stereo_node',
        name='stereo_vision_orchestrator',
        parameters=[
            config_front, 
            config_back,
            {'enabled_cameras':['front', 'back']} # ['front', 'back'] se quiser ativar ambos, ou ['front'] / ['back'] para apenas um
        ],
        output='screen'
    )
    nodes.append(stereo_node)

    # --- 2. TFs PARA CÂMERA FRONTAL ---
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_front_phys',
        arguments=['0.2', '0.0', '0.3', '0', '0', '0', 'base_link', 'front_camera_link']
    ))
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_front_opt',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'front_camera_link', 'front_camera_optical_link']
    ))

    # --- 3. TFs PARA CÂMERA TRASEIRA (Virada 180 graus / Pi radianos) ---
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_back_phys',
        # Exemplo: -0.2 (atrás), yaw = 3.1415 (virada para trás)
        arguments=['-0.2', '0.0', '0.3', '3.1415', '0', '0', 'base_link', 'back_camera_link']
    ))
    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_back_opt',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'back_camera_link', 'back_camera_optical_link']
    ))

    # --- 4. LASERSCAN FRONTAL ---
    nodes.append(Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_front',
        remappings=[
            ('depth', '/front_camera/depth/image_rect_raw'),
            ('depth_camera_info', '/front_camera/depth/camera_info'),
            ('scan', '/front_camera/scan')
        ],
        parameters=[{'range_max': 5.0, 'range_min': 0.3, 'scan_height': 3, 'output_frame': 'front_camera_link'}]
    ))

    # --- 5. LASERSCAN TRASEIRO ---
    nodes.append(Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_back',
        remappings=[
            ('depth', '/back_camera/depth/image_rect_raw'),
            ('depth_camera_info', '/back_camera/depth/camera_info'),
            ('scan', '/back_camera/scan')
        ],
        parameters=[{'range_max': 5.0, 'range_min': 0.3, 'scan_height': 3, 'output_frame': 'back_camera_link'}]
    ))

    return LaunchDescription(nodes)