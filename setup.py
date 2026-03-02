from setuptools import setup

package_name = 'frontier_explorer'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/frontier_explorer_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.com',
    description='FrontierFlow: Dynamic frontier-based autonomous exploration for ROS 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'frontier_explorer_node = frontier_explorer.frontier_explorer_node:main',
        ],
    },
)
