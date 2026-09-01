from setuptools import find_packages, setup

package_name = 'robot_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='meddahi',
    maintainer_email='meddahi@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'decision_node = robot_python.decision_node:main',
            'sensor_node = robot_python.sensor_node:main',
            'battery_node = robot_python.battery_node:main',
            'camera_node = robot_python.camera_node:main',
            'safety_node = robot_python.safety_node:main'
        ],
    },
)
