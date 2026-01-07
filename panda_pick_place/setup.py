from setuptools import find_packages, setup
import os                 
from glob import glob 

package_name = 'panda_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzz',
    maintainer_email='2748942425@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'arm_pymoveit2 = panda_pick_place.pymoveit2_arm:main',
        'arm_official = panda_pick_place.official_moveit2_arm:main',
        'pick_place_node = panda_pick_place.pick_place_node:main',
        ],
    },
)
