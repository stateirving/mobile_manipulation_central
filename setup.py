from glob import glob
from setuptools import find_packages, setup

package_name = 'mobile_manipulation_central'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where="src", exclude=['test']),
    package_dir={"": "src"},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf/xacro', glob('urdf/xacro/*.xacro')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/ur10', glob('config/ur10/*.yaml')),
        ('share/' + package_name + '/meshes', glob('meshes/*.dae')),
        ('share/' + package_name + '/meshes/ridgeback', glob('meshes/ridgeback/*')),
        ('share/' + package_name + '/meshes/ur10/collision', glob('meshes/ur10/collision/*')),
        ('share/' + package_name + '/meshes/ur10/visual', glob('meshes/ur10/visual/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benni',
    maintainer_email='benjamin.bogenberger@tum.de',
    description='Shared code for mobile manipulator experiments',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'ridgeback_state_publisher_node = mobile_manipulation_central.ridgeback_state_publisher_node:main',
        ],
    },
)
