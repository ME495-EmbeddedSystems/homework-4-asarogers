from setuptools import find_packages, setup

package_name = 'nubot_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/manual_explore.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/nubot.urdf.xacro']),
        ('share/' + package_name + '/config', ['config/nubot_urdf.rviz', 'config/ekf.yaml', 'config/nav2_params.yaml']),
        ('share/' + package_name + '/worlds', ['worlds/nubot_simple.sdf']),
        # ('share/' + package_name, ['nubot_nav/follow_waypoints.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='asaace00',
    maintainer_email='cyberasasoftware@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'follow_waypoints_node = nubot_nav.follow_waypoints:main'
        ],
    },
)
