from setuptools import setup
import os
from glob import glob

package_name = 'openmx'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include service files
        (os.path.join('share', package_name, 'srv'), 
            glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Forward and Inverse Kinematics for OpenManipulator-X',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'forward_kinematics = openmx.openmx_fwd:main',
            'inverse_kinematics = openmx.openmx_inv:main',
            'ik_client = openmx.ik_client:main',
        ],
    },
)