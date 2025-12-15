from setuptools import setup

package_name = 'virtual_environment'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装 launch 文件
        ('share/' + package_name + '/launch', ['launch/virtual_wall.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zwshao',
    maintainer_email='zwshao@todo.todo',
    description='Virtual objects (walls, spheres, cubes) for haptic rendering demo',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # ros2 run virtual_environment virtual_wall
            'virtual_wall = virtual_environment.virtual_wall_publisher:main',
            'collision_checker = virtual_environment.collision_checker:main',
            'force_feedback_node = virtual_environment.force_feedback_node:main',
        ],
    },
)
