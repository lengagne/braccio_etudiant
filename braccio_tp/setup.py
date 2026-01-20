from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'braccio_tp'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', 'braccio_tp', 'launch'),glob('launch/*.py')),
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
            'localize_objects=braccio_tp.localize_objects:main'
        ],
            # 'show_april_tags=braccio_tp.show_april_tags:main'
            # 'apriltag_pose=braccio_tp.apriltag_pose:main'
            # 'camera_calibrator=braccio_tp.camera_calibrator:main'
            # 'imaghe_converter=braccio_tp.imaghe_converter:main'
    },
)
