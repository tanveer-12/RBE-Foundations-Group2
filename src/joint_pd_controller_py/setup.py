from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'joint_pd_controller_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        # Include service files
        (os.path.join('share', package_name, 'srv'), glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tanveer12',
    maintainer_email='tanveer12@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pd_controller_node = joint_pd_controller_py.pd_controller:main',
            'test_client = joint_pd_controller_py.test_client:main',
            'plot_pd_results = joint_pd_controller_py.plot_pd_results:main',
        ],
    },
)
