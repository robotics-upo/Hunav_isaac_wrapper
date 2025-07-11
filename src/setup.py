from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'hunav_isaac_wrapper'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        
        # Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/**/*', recursive=True)),
        
        # Install scenario files
        (os.path.join('share', package_name, 'scenarios'),
            glob('scenarios/*.yaml')),
        
        # Install map files
        (os.path.join('share', package_name, 'maps'),
            glob('maps/*')),
        
        # Install world files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.usd') + glob('worlds/*.usd-cache/**/*', recursive=True)),
        
        # Install behavior tree files
        (os.path.join('share', package_name, 'behavior_trees'),
            glob('behavior_trees/*.xml')),
        
        # Install other files
        (os.path.join('share', package_name),
            ['README.md', 'isaacsim.exp.base.kit']),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'geometry_msgs',
        'std_msgs',
        'nav_msgs',
        'sensor_msgs',
        'tf2_ros',
        'tf2_geometry_msgs',
        'hunav_msgs',
        'pyyaml',
        'numpy',
        'matplotlib',
    ],
    zip_safe=True,
    maintainer='Miguel Escudero Jiménez',
    maintainer_email='mescjim@upo.es',
    description='Isaac Sim wrapper for HuNavSim human navigation simulation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hunav_isaac_main = scripts.main:main',
        ],
    },
)
