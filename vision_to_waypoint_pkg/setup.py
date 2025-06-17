from setuptools import find_packages, setup

package_name = 'vision_to_waypoint_pkg'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosuser',
    maintainer_email='vxlab@urbrain.vx.rmit.edu.au',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'waypoint_finder = vision_to_waypoint_pkg.waypoint_finder:main',
            'go_home_node = vision_to_waypoint_pkg.go_home_node:main',
            'go_to_object = vision_to_waypoint_pkg.go_to_object:main',
        ],
    },
)
