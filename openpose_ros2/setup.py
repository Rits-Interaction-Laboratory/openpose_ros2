from setuptools import setup

package_name = 'openpose_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Interaction Lab.',
    # TODO: OSS化のときに検討
    maintainer_email='yoshida@todo.todo',
    description='A ROS2 package that call the OpenPose from ROS2.',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'openpose_ros2 = openpose_ros2.openpose_node:main',
        ],
    },
)
