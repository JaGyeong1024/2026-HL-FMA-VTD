from setuptools import setup

package_name = 'vtd_autoware_bridge'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='HL FMA team',
    maintainer_email='wkrud4431@gmail.com',
    description='VTD TCP 9910 (HL FMA) <-> Autoware adapter: vehicle/localization/perception/traffic light + route injection',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = vtd_autoware_bridge.bridge_node:main',
            'route_node = vtd_autoware_bridge.route_node:main',
            'blocked_route_detour = vtd_autoware_bridge.blocked_route_detour:main',
            'pedestrian_proximity_slowdown = vtd_autoware_bridge.pedestrian_proximity_slowdown:main',
        ],
    },
)
