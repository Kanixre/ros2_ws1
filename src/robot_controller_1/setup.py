from setuptools import find_packages, setup

package_name = 'robot_controller_1'

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
    maintainer='kanixre',
    maintainer_email='offiongakanimo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "object_detector = robot_controller_1.object_detector1:main",
            "robot_nav = robot_controller_1.robot_nav1:main",
            "object_counter = robot_controller_1.object_counter1:main"
            
        ],
    },
)
