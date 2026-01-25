from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'braccio_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files (RViz, YAML, etc.)
        (os.path.join('share', package_name, 'config'),
            glob('config/*')),
        # URDF/Xacro files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),
        # Meshes (si vous en avez)
        (os.path.join('share', package_name, 'stl'),
            glob('stl/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='selengag',
    maintainer_email='lengagne@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
