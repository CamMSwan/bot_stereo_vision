import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. Resgate dos caminhos dos pacotes no Workspace
    pkg_stereo_vision = get_package_share_directory('bot_stereo_vision')
    
    try:
        pkg_realsense = get_package_share_directory('realsense2_camera')
        realsense_disponivel = True
    except Exception:
        realsense_disponivel = False

    # 2. Caminhos para os arquivos de configuração YAML das C920
    front_camera_config = os.path.join(pkg_stereo_vision, 'config', 'front_camera.yaml')
    back_camera_config = os.path.join(pkg_stereo_vision, 'config', 'back_camera.yaml')

    # 3. Declaração de Argumentos de Inicialização
    unidade_processamento = DeclareLaunchArgument(
        'use_gpu',
        default_value='true',
        description='Forçar o uso de aceleração por hardware (CUDA/TensorRT) na Jetson'
    )

    # Inicializa a lista unificada de nós e ações do launch
    launch_elements = [unidade_processamento]

    # --- 4. NÓ DE VISÃO ESTÉREO (ORQUESTRADOR MULTI-CÂMERA) ---
    stereo_vision_orchestrator_node = Node(
        package='bot_stereo_vision',
        executable='stereo_node',
        name='stereo_vision_orchestrator',
        output='screen',
        parameters=[
            front_camera_config,
            back_camera_config,
            {'enabled_cameras': ['front', 'back']}, # Ativa ambas as esteiras de câmeras C920
            {'use_gpu': LaunchConfiguration('use_gpu')}
        ],
        arguments=['--ros-args', '--log-level', 'info']
    )
    launch_elements.append(stereo_vision_orchestrator_node)

    # --- 5. TFs ESTÁTICAS PARA CÂMERA FRONTAL (Necessárias para o Laserscan) ---
    launch_elements.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_front_phys',
        arguments=['0.2', '0.0', '0.3', '0', '0', '0', 'base_link', 'front_camera_link']
    ))
    launch_elements.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_front_opt',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'front_camera_link', 'front_camera_optical_link']
    ))

    # --- 6. TFs ESTÁTICAS PARA CÂMERA TRASEIRA (Virada 180 graus / Pi radianos) ---
    launch_elements.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_back_phys',
        arguments=['-0.2', '0.0', '0.3', '3.1415', '0', '0', 'base_link', 'back_camera_link']
    ))
    launch_elements.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_back_opt',
        arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'back_camera_link', 'back_camera_optical_link']
    ))

    # --- 7. LASERSCAN FRONTAL (depthimage_to_laserscan) ---
    laserscan_front_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_front',
        remappings=[
            ('depth', '/front_camera/depth/image_rect_raw'),
            ('depth_camera_info', '/front_camera/depth/camera_info'),
            ('scan', '/front_camera/scan')
        ],
        parameters=[{'range_max': 5.0, 'range_min': 0.3, 'scan_height': 3, 'output_frame': 'front_camera_link'}]
    )
    launch_elements.append(laserscan_front_node)

    # --- 8. LASERSCAN TRASEIRO (depthimage_to_laserscan) ---
    laserscan_back_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='depthimage_to_laserscan_back',
        remappings=[
            ('depth', '/back_camera/depth/image_rect_raw'),
            ('depth_camera_info', '/back_camera/depth/camera_info'),
            ('scan', '/back_camera/scan')
        ],
        parameters=[{'range_max': 5.0, 'range_min': 0.3, 'scan_height': 3, 'output_frame': 'back_camera_link'}]
    )
    launch_elements.append(laserscan_back_node)

    # --- 9. CONFIGURAÇÃO DO LAUNCH DA INTEL REALSENSE (Se disponível no sistema) ---
    if realsense_disponivel:
        realsense_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_realsense, 'launch', 'rs_launch.py')
            ),
            launch_arguments={
                'enable_color': 'true',
                'enable_depth': 'true',
                'align_depth.enable': 'true',
                'pointcloud.enable': 'true',
                'rgb_camera.profile': '640x480x30', 
                'depth_module.profile': '640x480x30'
            }.items()
        )
        launch_elements.add_action(realsense_launch)
    else:
        # Erro de caractere de continuação de linha corrigido aqui de forma limpa
        print("[AVISO] Pacote realsense2_camera nao encontrado. Inicializando apenas o ecossistema C920 e Laserscans.")

    return LaunchDescription(launch_elements)