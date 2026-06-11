import cv2
import threading
import time
import subprocess
import sys
import numpy as np
from pathlib import Path

# Configuração de caminhos para localizar a raiz do pacote e a pasta 'core' na Jetson
SCRIPT_DIR = Path(__file__).resolve().parent
PACKAGE_ROOT = SCRIPT_DIR.parent

# Adiciona a raiz do pacote ao path para permitir importação de módulos internos se necessário
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

def apply_v4l2_config(device_id, cfg):
    """Aplica as configurações de hardware via v4l2-ctl usando o dicionário de parâmetros."""
    dev = f"/dev/video{device_id}"
    params = [
        f"auto_exposure={cfg['auto_exposure']}",
        f"exposure_time_absolute={cfg['exposure_time_absolute']}",
        f"exposure_dynamic_framerate={cfg['exposure_dynamic_framerate']}",
        f"gain={cfg['gain']}",
        f"brightness={cfg['brightness']}",
        f"contrast={cfg['contrast']}",
        f"saturation={cfg['saturation']}",
        f"backlight_compensation={cfg['backlight_compensation']}",
        f"power_line_frequency={cfg['power_line_frequency']}",
        f"white_balance_automatic={cfg['white_balance_automatic']}",
        f"white_balance_temperature={cfg['white_balance_temperature']}",
        f"focus_automatic_continuous={cfg['focus_automatic_continuous']}",
        f"focus_absolute={cfg['focus_absolute']}"
    ]
    
    cmd = ["v4l2-ctl", "-d", dev]
    for p in params:
        cmd.extend(["-c", p])
    
    try:
        subprocess.run(cmd, check=True, capture_output=True)
    except Exception as e:
        print(f"[ERRO] Falha ao configurar {dev}: {e}")

class FrameGrabberStereo:
    def __init__(self, position, ros_params):
        self.pos = position.lower()
        self.cfg = ros_params  # Dicionário de parâmetros vindo do YAML
        
        # IDs e dimensões
        self.ids = [self.cfg['cam_left_id'], self.cfg['cam_right_id']]
        self.tw, self.th = self.cfg['target_width'], self.cfg['target_height']
        
        # Intervalo baseado no FPS do YAML
        self.intervalo = 1.0 / self.cfg['fps']

        # Carregamento de Mapas de Calibração
        self._load_maps()

        # Configuração de Padding (Múltiplos de 32 para o modelo)
        pad_h = (32 - (self.th % 32)) % 32
        self.top, self.bottom = pad_h // 2, pad_h - (pad_h // 2)
        pad_w = (32 - (self.tw % 32)) % 32
        self.left, self.right = pad_w // 2, pad_w - (pad_w // 2)
        self.out_w, self.out_h = self.tw + pad_w, self.th + pad_h

        # Estado da Thread
        self.running = False
        self.lock = threading.Lock()
        self.frame_ready = threading.Event()
        self.frames_out = []
        self.visual_frame_out = None
        
        # Inicialização do CLAHE para normalização
        self.clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))

    def _load_maps(self):
        """Carrega os arquivos .xml da pasta config do pacote."""
        map_path = PACKAGE_ROOT / "config" / self.cfg['map_file']
        if not map_path.exists():
            raise FileNotFoundError(f"Mapa estéreo não encontrado: {map_path}")

        cv_file = cv2.FileStorage(str(map_path), cv2.FILE_STORAGE_READ)
        self.m_Lx = cv_file.getNode('stereoMapL_x').mat()
        self.m_Ly = cv_file.getNode('stereoMapL_y').mat()
        self.m_Rx = cv_file.getNode('stereoMapR_x').mat()
        self.m_Ry = cv_file.getNode('stereoMapR_y').mat()
        cv_file.release()

    def _apply_clahe(self, img):
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        l, a, b = cv2.split(lab)
        l = self.clahe.apply(l)
        return cv2.cvtColor(cv2.merge((l, a, b)), cv2.COLOR_LAB2BGR)

    def start(self):
        """Aplica V4L2, abre capturas e inicia a thread."""
        for idx in self.ids:
            apply_v4l2_config(idx, self.cfg)

        self.capL = cv2.VideoCapture(self.ids[0], cv2.CAP_V4L2)
        self.capR = cv2.VideoCapture(self.ids[1], cv2.CAP_V4L2)

        for cap in [self.capL, self.capR]:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg['width'])
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg['height'])
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.running = True
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return self

    def _run(self):
        """Loop de captura e pré-processamento."""
        while self.running:
            t_inicio = time.perf_counter()
            
            if self.capL.grab() and self.capR.grab():
                retL, fL = self.capL.retrieve()
                retR, fR = self.capR.retrieve()

                if retL and retR:
                    try:
                        # 1. Retificação via mapas OpenCV
                        rectL = cv2.remap(fL, self.m_Lx, self.m_Ly, cv2.INTER_LINEAR)
                        rectR = cv2.remap(fR, self.m_Rx, self.m_Ry, cv2.INTER_LINEAR)
                        
                        #high_res_color = rectL #imagem 640x480 da left
                        
                        processed = []
                        for img in [rectL, rectR]:
                            # 2. Normalização e Redimensionamento
                            img_norm = self._apply_clahe(img)
                            rsz = cv2.resize(img_norm, (self.tw, self.th))
                            # 3. Padding para o FoundationStereo
                            pad = cv2.copyMakeBorder(rsz, self.top, self.bottom, self.left, self.right, 
                                                   cv2.BORDER_CONSTANT, value=[0, 0, 0])
                            processed.append(pad)

                        with self.lock:
                            self.frames_out = processed 
                            
                        # --- NOVA LÓGICA DE VISUALIZAÇÃO ADICIONADA ---
                        img_visual_norm = self._apply_clahe(rectL)
                        # Redimensiona para a nova resolução de visualização (640x360)
                        self.visual_frame_out = cv2.resize(img_visual_norm, (640, 360))
                        
                        self.frame_ready.set()
                        
                    except Exception as e:
                        print(f"[ERRO] Loop de captura ({self.pos}): {e}")

            t_proc = time.perf_counter() - t_inicio
            time.sleep(max(0, self.intervalo - t_proc))

    def read(self):
        """Retorna o par de frames processados originais para a IA."""
        if self.frame_ready.wait(timeout=2.0):
            self.frame_ready.clear()
            with self.lock:
                return True, [f.copy() for f in self.frames_out]
        return False, []

    def read_visual(self):
        """Retorna o frame retificado e redimensionado para visualização (640x360)."""
        with self.lock:
            if self.visual_frame_out is not None:
                return self.visual_frame_out.copy()
        return None

    def release(self):
        """Encerra os recursos de hardware."""
        self.running = False
        time.sleep(0.2)
        if hasattr(self, 'capL'): self.capL.release()
        if hasattr(self, 'capR'): self.capR.release()