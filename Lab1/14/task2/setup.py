from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #(os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='me597',
    maintainer_email='pgery@purdue.edu',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = task2.JointPub:main',
            'listener = task2.JointSub:main', 
            'service = task2.JointService:main',
            'client = task2.JointClient:main',
        ],
    },
)
