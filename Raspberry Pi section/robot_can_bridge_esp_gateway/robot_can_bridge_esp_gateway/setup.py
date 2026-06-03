from setuptools import setup

package_name = 'robot_can_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/bridge_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='pi@example.com',
    description='CAN bridge and simple checkpoint follower',
    license='MIT',
    entry_points={
        'console_scripts': [
            'can_bridge = robot_can_bridge.can_bridge_node:main',
            'checkpoint_follower = robot_can_bridge.checkpoint_follower_node:main',
        ],
    },
)
