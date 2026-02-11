from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'braccio_common'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob('launch/*.py')),
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
            'aruco_to_tag_converter=braccio_common.aruco_to_tag_converter:main',
            'image_converter=braccio_common.image_converter:main',
            'camera_calibrator=braccio_common.camera_calibrator:main',
            'pad_to_target=braccio_common.pad_to_target:main',
        ],
    },
)
