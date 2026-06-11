import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage 
from cv_bridge import CvBridge
import numpy as np
import os
import sys
from ament_index_python.packages import get_package_share_directory
import cv2

# Ajuste de PATH para importar as classes da pasta 'scripts' no ambiente instalado
pkg_share = get_package_share_directory('bot_stereo_vision')
scripts_path = os.path.join(pkg_share, 'scripts')
if scripts_path not in sys.path:
    sys.path.insert(0, scripts_path)

# Importação das classes modulares
from depth_processor import DepthProcessor

class StereoVisionNode(Node):
    def __init__(self):
        super().__init__('stereo_vision_node')
        self.bridge = CvBridge()

        # 1. Torna a lista de câmeras dinâmica via parâmetro do ROS
        # Se você não passar nada no Launch, ele tenta carregar a 'front'
        self.declare_parameter('enabled_cameras', ['front']) 
        camera_positions = self.get_parameter('enabled_cameras').value
        
        self.cameras = {}
        pkg_share = get_package_share_directory('bot_stereo_vision')
        
        # Valor padrão caso nenhuma câmera carregue
        self.node_fps = 5 

        for pos in camera_positions:
            self.get_logger().info(f"Iniciando configuração da câmera: {pos.upper()}")
            
            # Tenta carregar os parâmetros. Se falhar (ex: falta de YAML), pula
            try:
                cam_params = self._get_params_for_camera(pos)
                # Só atualiza o FPS se o valor for válido (> 0)
                fps_val = float(cam_params.get('fps', 5.0))
                if fps_val > 0.1:
                    self.node_fps = fps_val
                else:
                    self.node_fps = 5.0  # Fallback de segurança
            except Exception as e:
                self.get_logger().warn(f"Parâmetros para {pos} não encontrados. Pulando.")
                continue

            self.cameras[pos] = {}
            
            ## 1.1 Carregar Matriz de Calibração do XML (Específico para cada posição)
            map_file = f"stereo_map_{pos}.xml"
            xml_path = os.path.join(pkg_share, 'config', map_file)
            
            fs = cv2.FileStorage(xml_path, cv2.FILE_STORAGE_READ)
            if not fs.isOpened():
                self.get_logger().error(f"Não foi possível abrir o ficheiro XML para {pos}: {xml_path}")
                continue
                
            q_node = fs.getNode("Q")
            if q_node.empty():
                self.get_logger().error(f"A matriz 'Q' não foi encontrada no XML de {pos}!")
                fs.release()
                continue
                
            q_matrix = q_node.mat()
            fs.release()

            # Extração da geometria do XML
            self.cameras[pos]['f_orig'] = q_matrix[2, 3]
            self.cameras[pos]['cx_orig'] = -q_matrix[0, 3]
            self.cameras[pos]['cy_orig'] = -q_matrix[1, 3]
            
            # 1.2 Publishers para cada câmera (Namespace dinâmico)
            self.cameras[pos]['pub_rgb'] = self.create_publisher(Image, f'/{pos}_camera/color/image_raw', 10)
            self.cameras[pos]['pub_info_rgb'] = self.create_publisher(CameraInfo, f'/{pos}_camera/color/camera_info', 10)
            self.cameras[pos]['pub_depth_rtab'] = self.create_publisher(Image, f'/{pos}_camera/aligned_depth_to_color/image_raw', 10)
            self.cameras[pos]['pub_depth_laser'] = self.create_publisher(Image, f'/{pos}_camera/depth/image_rect_raw', 10)
            self.cameras[pos]['pub_info_laser'] = self.create_publisher(CameraInfo, f'/{pos}_camera/depth/camera_info', 10)
            
            # Create publisher for front camera compressed image
            self.publisher_compressed = self.create_publisher(CompressedImage, 'compressed_camera', 10)
            # Create publisher for back camera compressed image
            self.publisher_compressed_back = self.create_publisher(CompressedImage, 'compressed_back_camera', 10)

            # 3. Inicialização do Processor (TensorRT)
            try:
                self.cameras[pos]['processor'] = DepthProcessor(pos, cam_params)
                self.get_logger().info(f"[{pos.upper()}] Pipeline TensorRT carregada.")
            except Exception as e:
                self.get_logger().error(f"Falha ao iniciar Pipeline para {pos}: {e}")
                del self.cameras[pos]

        # 4. Timer Global (Inicia o loop após configurar todas as câmeras)
        self.timer = self.create_timer(1.0/self.node_fps, self.pipeline_callback)
        self.get_logger().info(f"Nó de visão multi-câmera ativo a {self.node_fps} FPS.")

    def _get_params_for_camera(self, prefix):
        """Lê os parâmetros do YAML usando o prefixo fornecido."""
        string_params = ['position', 'map_file']
        param_names = [
            'position', 'baseline', 'focal_length', 'min_depth', 'max_depth',
            'cam_left_id', 'cam_right_id', 'width', 'height', 'fps',
            'target_width', 'target_height', 'map_file',
            'auto_exposure', 'exposure_time_absolute', 'exposure_dynamic_framerate',
            'gain', 'brightness', 'contrast', 'saturation',
            'backlight_compensation', 'power_line_frequency',
            'white_balance_automatic', 'white_balance_temperature',
            'focus_automatic_continuous', 'focus_absolute'
        ]
        
        params_dict = {}
        for name in param_names:
            full_param_name = f"{prefix}.{name}"
            
            if name in string_params:
                self.declare_parameter(full_param_name, "default_value")
            elif any(x in name for x in ['id', 'auto', 'width', 'height', 'frequency', 'absolute', 'compensation', 'dynamic', 'gain', 'brightness', 'contrast', 'saturation', 'backlight', 'power_line', 'white_balance', 'focus']):
                self.declare_parameter(full_param_name, 0)
            else:
                self.declare_parameter(full_param_name, 0.0)
            
            params_dict[name] = self.get_parameter(full_param_name).value
        
        return params_dict

    def pipeline_callback(self):
        # Timestamp único para garantir sincronia temporal entre todos os sensores
        timestamp = self.get_clock().now().to_msg()

        for pos, data in self.cameras.items():
            if 'processor' not in data:
                continue

            result = data['processor'].get_frame()
            if result is None:
                continue

            depth_map, visual_frame = result
            frame_id = f"{pos}_camera_optical_link"

            # Converte profundidade de metros para milímetros uint16
            depth_mm = np.nan_to_num(depth_map * 1000, nan=0).astype(np.uint16)
            
            # Geração das mensagens
            msg_rgb = self.bridge.cv2_to_imgmsg(visual_frame, "bgr8")
            msg_depth = self.bridge.cv2_to_imgmsg(depth_mm, "16UC1")
            msg_info = self._generate_camera_info(timestamp, pos)
            
            # Fluxo de Visualização Comprimida (640x360) integrado ao padrão
            comp_msg = None
            high_res_visual = data['processor'].grabber.read_visual()
            
            if high_res_visual is not None:
                ret_enc, jpeg_buffer = cv2.imencode('.jpg', high_res_visual, [int(cv2.IMWRITE_JPEG_QUALITY), 75])
                if ret_enc:
                    comp_msg = CompressedImage()
                    comp_msg.format = "jpeg"
                    comp_msg.data = jpeg_buffer.tobytes()

            # Agrupamento de mensagens para aplicação do Header em lote (Stamp + Frame ID)
            active_messages = [msg_rgb, msg_depth, msg_info]
            if comp_msg is not None:
                active_messages.append(comp_msg)

            # Aplicação do Header (Stamp + Frame ID)
            for m in [msg_rgb, msg_depth, msg_info]:
                m.header.stamp = timestamp
                m.header.frame_id = frame_id

            # Publicação nos tópicos da câmera atual
            data['pub_rgb'].publish(msg_rgb)
            data['pub_info_rgb'].publish(msg_info)
            data['pub_depth_rtab'].publish(msg_depth)
            data['pub_depth_laser'].publish(msg_depth)
            data['pub_info_laser'].publish(msg_info)
        
    def _generate_camera_info(self, stamp, pos):
        """Calcula a matriz K e P ajustada pela escala da imagem atual."""
        msg = CameraInfo()
        msg.header.stamp = stamp # Garante o carimbo de tempo correto
        
        tw = self.get_parameter(f"{pos}.target_width").value
        th = self.get_parameter(f"{pos}.target_height").value
        msg.width = tw
        msg.height = th

        # Ajuste de escala para a focal lida da calibração (baseada em 640px)
        scale = tw / 640.0 
        f = self.cameras[pos]['f_orig'] * scale
        cx = self.cameras[pos]['cx_orig'] * scale
        cy = self.cameras[pos]['cy_orig'] * scale

        msg.distortion_model = "plumb_bob"
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.k = [f, 0.0, cx, 0.0, f, cy, 0.0, 0.0, 1.0]
        msg.p = [f, 0.0, cx, 0.0, 0.0, f, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        return msg

def main(args=None):
    rclpy.init(args=args)
    node = StereoVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Garante a liberação limpa do hardware da Jetson para ambas as câmeras
        if hasattr(node, 'cameras'):
            for pos, data in node.cameras.items():
                if 'processor' in data:
                    node.get_logger().info(f"Liberando hardware da câmera: {pos.upper()}")
                    data['processor'].release()
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()