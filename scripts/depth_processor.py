import numpy as np
import cv2
import torch
import os
import sys
import yaml
from pathlib import Path
from omegaconf import OmegaConf

# 1. Configuração de caminhos (Focado no diretório 'share' do ROS)
SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = SCRIPT_DIR.parent

# Adiciona a pasta scripts ao path para importações diretas
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

# Caminho para o repositório clonado
MODEL_PATH_ROOT = PACKAGE_ROOT / "model" / "Fast-FoundationStereo"

# Adiciona a raiz do modelo ao path para o core e Utils serem encontrados
if str(MODEL_PATH_ROOT) not in sys.path:
    sys.path.insert(0, str(MODEL_PATH_ROOT))

# Importações Absolutas (Mais seguras no ambiente ROS)
from frame_grabber_stereo import FrameGrabberStereo
from core.foundation_stereo import TrtRunner

class DepthProcessor:
    def __init__(self, position, ros_params):
        self.position = position.lower()
        self.cfg = ros_params
        
        self.tw = self.cfg['target_width']
        self.th = self.cfg['target_height']
            
        # Caminho da pasta output onde estão os engines
        onnx_dir = str(MODEL_PATH_ROOT / "output")
        yaml_path = os.path.join(onnx_dir, "onnx.yaml")
        
        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"[ERRO] onnx.yaml não encontrado em: {yaml_path}")

        with open(yaml_path, "r") as f:
            model_cfg = yaml.safe_load(f)
        args = OmegaConf.create(model_cfg)
        
        print(f"[BOTBOT] Carregando Motores TensorRT para {self.position.upper()}...")
        # Certifique-se que os nomes dos arquivos engine batem com o que está na pasta
        self.model = TrtRunner(
            args, 
            os.path.join(onnx_dir, "feature_runner.engine"), 
            os.path.join(onnx_dir, "post_runner.engine")
        )

        self.grabber = FrameGrabberStereo(self.position, self.cfg).start()
        self.focal = self._get_focal()

    def _get_focal(self):
        map_path = PACKAGE_ROOT / "config" / self.cfg['map_file']
        
        if not map_path.exists():
            print(f"[ERRO] Calibração não encontrada em: {map_path}")
            return 500.0
            
        cv_file = cv2.FileStorage(str(map_path), cv2.FILE_STORAGE_READ)
        q_node = cv_file.getNode('Q')
        if q_node.empty():
            cv_file.release()
            return 500.0
            
        focal = float(q_node.mat()[2, 3]) * (self.tw / self.cfg['width'])
        cv_file.release()
        return focal


    def _to_depth(self, disp):
        """Converte disparidade em profundidade métrica e gera mapa de calor."""
        with np.errstate(divide='ignore', invalid='ignore'):
            # Profundidade = (B * f) / d
            depth = (float(self.cfg['baseline']) * self.focal) / disp
            
            #depth_corrigido = -0.0263*(depth*depth) + 1.0064*depth -0.0091 #(front)
            depth_corrigido = -0.0246*(depth*depth) + 0.9713*depth -0.0018 #(back)
            # Filtra valores fora do range configurado
            min_d = self.cfg['min_depth']
            max_d = self.cfg['max_depth']
            depth_corrigido[(depth_corrigido < min_d) | (depth_corrigido > max_d)] = np.nan
        
        return depth_corrigido

    def get_frame(self):
        """Lê os frames do hardware e realiza a inferência."""
        ok, frames = self.grabber.read()
        if not ok:
            return None

        # Preparação dos tensores (C920 L/R) para GPU
        x0 = torch.from_numpy(frames[0]).permute(2, 0, 1).float().unsqueeze(0).cuda()
        x1 = torch.from_numpy(frames[1]).permute(2, 0, 1).float().unsqueeze(0).cuda()

        # Inferência via TensorRT
        disp_tensor = self.model(x0, x1)
        
        # Remoção de Padding adicionado para o FoundationStereo (múltiplos de 32)
        disp_full = disp_tensor.detach().cpu().numpy().reshape(self.grabber.out_h, self.grabber.out_w)
        t, l = self.grabber.top, self.grabber.left
        disp_final = disp_full[t : t + self.th, l : l + self.tw].clip(0, None)

        # Recorte da Imagem RGB Real (Câmera Esquerda)
        # Aplicamos o mesmo t e l para remover o padding e alinhar com o depth_map
        rgb_rect = frames[0][t : t + self.th, l : l + self.tw]

        # Conversão para metros e visualização térmica
        depth_map= self._to_depth(disp_final)
        
        # RETORNO CORRIGIDO: Retornamos o mapa métrico e a imagem RGB real
        return [depth_map, rgb_rect]
    def release(self):
        """Encerra a captura de hardware."""
        self.grabber.release()