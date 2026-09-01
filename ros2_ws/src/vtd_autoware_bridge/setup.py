from setuptools import setup

package_name = 'vtd_autoware_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bridge.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='VTD TCP 9910 to Autoware topic bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'bridge_node = vtd_autoware_bridge.bridge_node:main',
        ],
    },
)
