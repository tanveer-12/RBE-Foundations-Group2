# from setuptools import setup
# import os
# from glob import glob

# package_name = 'openmx'

# setup(
#     name=package_name,
#     version='0.0.0',
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         # Include service files
#         (os.path.join('share', package_name, 'srv'), 
#             glob('srv/*.srv')),
#     ],
#     install_requires=['setuptools'],
#     zip_safe=True,
#     maintainer='group_2',
#     maintainer_email='group_2@example.com',
#     description='Forward and Inverse Kinematics for OpenManipulator-X',
#     license='TODO: License declaration',
#     tests_require=['pytest'],
#     entry_points={
#         'console_scripts': [
#             'forward_kinematics = openmx.openmx_fwd:main',
#             'inverse_kinematics = openmx.openmx_inv:main',
#             'ik_client = openmx.ik_client:main',
#         ],
#     },
# )


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

        # Install all service files
        (os.path.join('share', package_name, 'srv'),
            glob('srv/*.srv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='group_2',
    maintainer_email='group_2@example.com',
    description='Forward, Inverse & Velocity Kinematics for OpenManipulator-X',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'openmx_fwd = openmx.openmx_fwd:main',
            'openmx_inv = openmx.openmx_inv:main',
            'incremental_controller = openmx.openmx_increment_pos_ref:main',
            'ik_client = openmx.ik_client:main',
            'openmx_jacobian = openmx.openmx_jacobian:main',   # (optional: since you have openmx_jacobian.py)
            'endeff_node = openmx.openmx_publish_endeff:main',   # (optional: since you have openmx_jacobian.py)
        ],
        # 'console_scripts': [
        #     'forward_kinematics = openmx.openmx_fwd:main',
        #     'inverse_kinematics = openmx.openmx_inv:main',
        #     'increment_pos_ref = openmx.openmx_increment_pos_ref:main',
        #     'ik_client = openmx.ik_client:main',
        #     'jacobian_node = openmx.openmx_jacobian:main',   # (optional: since you have openmx_jacobian.py)
        #     'endeff_node = openmx.openmx_publish_endeff:main',   # (optional: since you have openmx_jacobian.py)
        # ],
    },
)

