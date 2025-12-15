from setuptools import find_packages, setup

package_name = 'omni_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/touch_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zwshao',
    maintainer_email='zwshao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'touch_hybrid_pose_control = omni_control.touch_hybrid_pose_control_v1:main',
            'touch_hybrid_pose_control = omni_control.touch_hybrid_pose_control_v2:main',
            'touch_control_visualizer = omni_control.touch_control_visualizer:main',
        ],
    },
)
