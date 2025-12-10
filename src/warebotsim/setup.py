from setuptools import find_packages, setup
import os
from glob import glob
from setuptools import setup
from setuptools import find_packages


package_name = 'warebotsim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name), glob('urdf/jackal.urdf')),
        (os.path.join('share', package_name), glob('urdf/r2d2.rviz')),
        (os.path.join('share', package_name, 'meshes'), glob('urdf/meshes/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yektapanj',
    maintainer_email='yektapanj@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'state_publisher = warebotsim.state_publisher:main'
        ],
    },
)
