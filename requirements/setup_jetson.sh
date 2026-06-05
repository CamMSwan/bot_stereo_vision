#!/bin/bash
# Script de Pareamento de Hardware - Jetson Orin Nano (JetPack 6.x)
# Projeto Capstone Insper - bot_stereo_vision
set -e

# Garante que o script está executando a partir da pasta onde ele está salvo
cd "$(dirname "$0")"

echo "======================================================="
echo "  BotBot: Iniciando Instalação de Pré-requisitos CUDA  "
echo "======================================================="

echo "--> 1. Atualizando repositórios e dependências nativas..."
sudo apt update && sudo apt install -y v4l-utils libjpeg-dev zlib1g-dev libpython3-dev libavcodec-dev libavformat-dev libswscale-dev wget git

echo "--> 2. Removendo possíveis resíduos de CPU do Pip..."
pip3 uninstall torch torchvision tensorrt -y || true

echo "--> 3. Baixando e Instalando PyTorch 2.3.0 oficial NVIDIA (CUDA)..."
wget https://nvidia.box.com/shared/static/zvultzsmd4iuheykxy17s4l2n91ylpl8.whl -O torch-2.3.0-cp310-cp310-linux_aarch64.whl
pip3 install torch-2.3.0-cp310-cp310-linux_aarch64.whl --user

echo "--> 4. Baixando e Instalando Torchvision 0.18.0 oficial NVIDIA (CUDA)..."
wget https://nvidia.box.com/shared/static/u0ziu01c0kyji4zz3gxam79181nebylf.whl -O torchvision-0.18.0-cp310-cp310-linux_aarch64.whl
pip3 install torchvision-0.18.0-cp310-cp310-linux_aarch64.whl --user

echo "--> 5. Limpando arquivos temporários (.whl)..."
rm torch-2.3.0-cp310-cp310-linux_aarch64.whl
rm torchvision-0.18.0-cp310-cp310-linux_aarch64.whl

echo "--> 6. Chamando automaticamente a esteira do requirements.txt..."
pip3 install -r requirements.txt --user

echo "======================================================="
echo "       AMBIENTE SUBIDO E CONFIGURADO COM SUCESSO!      "
echo "======================================================="
# Teste rápido final de verificação de hardware
python3 -c "import torch; print('-> PyTorch OK! CUDA disponível:', torch.cuda.is_available())"