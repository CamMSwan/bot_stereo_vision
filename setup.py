from setuptools import setup
import os
from glob import glob

package_name = 'bot_stereo_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Launch e Config
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        
        # Scripts Modulares
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        
        # --- CORREÇÃO PARA O MODELO CLONADO ---
        # Precisamos instalar cada subpasta vital separadamente
        (os.path.join('share', package_name, 'model/Fast-FoundationStereo/output'), 
         glob('model/Fast-FoundationStereo/output/*')),
        
        (os.path.join('share', package_name, 'model/Fast-FoundationStereo/core'), 
         glob('model/Fast-FoundationStereo/core/*.py')),
        
        (os.path.join('share', package_name, 'model/Fast-FoundationStereo'), 
         glob('model/Fast-FoundationStereo/Utils.py')),

        # Se houver subpastas dentro de core ou Utils, você deve adicioná-las aqui também
        (os.path.join('share', package_name, 'model/Fast-FoundationStereo/core/utils'), 
         glob('model/Fast-FoundationStereo/core/utils/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='botbot',
    maintainer_email='botbot@todo.todo',
    description='Pipeline de visão estéreo modular para o BotBot',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'stereo_node = bot_stereo_vision.stereo_vision_node:main',
        ],
    },
)