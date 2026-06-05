# Módulo de Visão Estéreo do BotBot (`bot_stereo_vision`)

**Projeto CAPSTONE - Insper (Semestre 2026.1)** *Alternativa de Visão Estéreo de Baixo Custo para Navegação Autônoma de Robôs.*

---

## 1. Introdução

Este repositório contém o pacote ROS 2 `bot_stereo_vision`, desenvolvido como projeto de fim de curso (Capstone/TCC) na Engenharia do Insper no semestre 2026.1. 

O objetivo central do projeto é projetar, implementar e validar um sistema de visão estéreo profundo de baixo custo (utilizando duas câmeras Logitech C920 convencionais) capaz de atuar como uma alternativa acessível e eficiente a sensores de profundidade comerciais (como sensores LiDAR e câmeras RGB-D dedicadas). O sistema reconstrói o ambiente tridimensional e gera mapas de profundidade acelerados por hardware diretamente na GPU, fornecendo dados cruciais de percepção para a navegação autônoma e evasão de obstáculos do robô **BotBot**.

---

## 2. Estrutura do Repositório

A organização do repositório foi projetada para manter o pacote do ROS 2 completamente isolado de scripts de pareamento de ambiente externo, garantindo portabilidade entre workspaces.

```text
bot_stereo_vision/                  # Raiz do pacote (deve ser clonado dentro de botbrain_ws/src/)
├── bot_stereo_vision/              # Diretório de código-fonte principal do módulo Python
│   ├── __init__.py                 # Inicializador do pacote Python
│   └── stereo_vision_node.py       # Nó principal do ROS 2 (Orquestrador da Pipeline Estéreo)
├── config/                         # Parâmetros de calibração e inicialização dos sensores
│   ├── back_camera.yaml            # Configuração de tópicos/parâmetros da câmera traseira
│   ├── front_camera.yaml           # Configuração de tópicos/parâmetros da câmera frontal
│   ├── stereo_map_back.xml         # Matrizes de retificação intrínseca/extrínseca (Câmera Traseira)
│   └── stereo_map_front.xml        # Matrizes de retificação intrínseca/extrínseca (Câmera Frontal)
├── launch/                         # Scripts de automação de inicialização de nós
│   └── C920_cameras.launch.py      # Launch file oficial que inicializa o pipeline de visão
├── model/                          # Diretório reservado para o modelo Fast-FoundationStereo (Deve ser criada pelo usuario)
├── requirements/                   # Infraestrutura e governança de ambiente externo ao ROS
│   ├── requirements.txt            # Lista de dependências necessárias para rodar o projeto. 
│   └── setup_jetson.sh             # Script Bash para injeção limpa de pacotes CUDA da NVIDIA e instala as dependencias
├── resource/                       # Arquivo de indexação do ecossistema ROS
│   └── bot_stereo_vision           # Registro de index para descoberta do pacote pelo 'ament'
├── scripts/                        # Classes auxiliares e algoritmos puros de visão computacional
│   ├── __init__.py                 # Permite importação local de módulos
│   ├── depth_processor.py          # Processador e gerador do mapa de correlação e profundidade
│   └── frame_grabber_stereo.py     # Captura sincronizada de frames via V4L2/OpenCV
├── .gitignore                      # Filtro de arquivos descartáveis para o controle de versão (Git)
├── package.xml                     # Metadados do pacote e declaração de dependências do ROS 2
├── README.md                       # Documentação técnica oficial do sistema
├── setup.cfg                       # Configuração de diretórios de instalação para o setuptools
└── setup.py                        # Script de build do colcon e mapeamento de console_scripts 
```

---

## 3. Pré-Requisitos

Para que o pipeline de inteligência artificial e processamento estéreo rode em tempo real, o ambiente de hardware e software deve seguir rigidamente as especificações abaixo.

### 3.1. Especificações de Sistema
* **Hardware Alvo:** NVIDIA Jetson Orin Nano (Developer Kit ou placa base customizada).
* **Sensores:** 2x ou 4x Câmeras USB Logitech C920 montadas em um ou duas base estéreo rígida.
* **Sistema Operacional:** Ubuntu 22.04 LTS com **JetPack 6.x** (L4T 36.5.0).
* **Middleware:** ROS 2 Humble Hawksbill (Instalação padrão via `apt`).

### 3.2. Matriz de Dependências do Ambiente (Python/CUDA)
Devido a restrições de arquitetura ARM64 e barramentos de aceleração por hardware, a instalação de dependências de IA via repositório global do Python (`PyPI`) padrão instala pacotes puramente de CPU, inviabilizando o projeto. 

O ambiente deve ser estabilizado utilizando a seguinte matriz de versões:
* **PyTorch:** `2.3.0` (Compilado oficialmente pela NVIDIA com suporte a CUDA).
* **Torchvision:** `0.18.0` (Vinculado nativamente aos kernels de CUDA).
* **OpenAI Triton:** `3.5.0` (Necessário para a compilação local dos volumes de correlação por grupo - GWC).
* **NumPy:** `>=1.21.5, <2.0` (Travado na árvore 1.x para manter compatibilidade com o ROS `cv_bridge` e evitar quebras de C-API).
* **OpenCV Python:** `4.9.0.80` (Versão compatível com a árvore NumPy 1.x).
* **TensorRT:** `10.3.0` (Injetado via pacotes nativos do JetPack).

### 3.3. Preparação Automatizada do Hardware
Antes de realizar o build do workspace do ROS 2, as dependências de hardware devem ser injetadas localmente. O script incluído na pasta `requirements/` automatiza de forma limpa todo o processo (limpeza de resíduos de CPU, download de wheels de 1.1GB da NVIDIA e Cargo/Pip optimization).

Para rodar a automação, execute no terminal da Jetson:

```bash
cd ~/BotBrain/botbrain_ws/src/bot_stereo_vision/requirements/
chmod +x setup_jetson.sh
./setup_jetson.sh
```
### 3.4. Instalação do modelo
Agora é necessário criar uma pasta `model/` dentro da estrutura principal, onde voce importará o modelo FastFoundation-Stereo. 
