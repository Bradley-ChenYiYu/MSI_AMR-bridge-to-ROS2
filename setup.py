'''
ref:
https://www.theconstruct.ai/ros2-qa-215-how-to-use-ros2-python-launch-files/
'''
from setuptools import find_packages, setup
import os
from glob import glob
# Cython
from Cython.Build import cythonize



package_name = 'player_bridge'
# Cython
files = package_name + "/*.py"
# files = [package_name + "/*.py", "launch/*.py"]

setup(
    # Cython
    ext_modules=cythonize(files,compiler_directives={'language_level' : "3"},force=True,quiet=True),
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/environment', ['environment/99_player_lib.sh']),
        (os.path.join('share', package_name, 'client_lib'), glob('client_lib/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    
    
    
    # Cython
    install_requires=['setuptools', "wheel",  "Cython"],
    zip_safe=True,
    author='leonli',
    author_email='leonli@msi.com',
    maintainer='leonli',
    maintainer_email='leonli@msi.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 bridge for Player/Stage robot API',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'player_bridge = player_bridge.player_bridge_node:main',
            'player_rgb_bridge = player_bridge.player_rgb_bridge_node:main',
            'player_battery_bridge = player_bridge.player_battery_bridge_node:main',
            'laser_merger_node = player_bridge.laser_merger_node:main',
        ],
    },
)
