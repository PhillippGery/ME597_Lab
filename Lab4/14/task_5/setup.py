from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_5'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'resource'), glob('resource/*')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gery',
    maintainer_email='pgery@purdue.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'image_publisher = task_5.image_publisher:main',
            'object_detector = task_5.object_detector:main',
            
        ],
    },
)
