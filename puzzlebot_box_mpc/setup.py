from setuptools import find_packages, setup
from glob import glob

package_name = 'puzzlebot_box_mpc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	    ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='puzzlebot',
    maintainer_email='puzzlebot@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'puzzlebot_odometry = puzzlebot_box_mpc.puzzlebot_odometry:main',
            'mpc_hw = puzzlebot_box_mpc.mpc_hw:main',
            'calibration_node =  puzzlebot_box_mpc.calibration_node:main',
            'teleop =  puzzlebot_box_mpc.teleop:main',
        ],
    },
)
