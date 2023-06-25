import os
from glob import glob
from setuptools import setup

package_name = 'rescue_quadrotor_exercise'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name), glob('config/*config.rviz'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='schwegmann',
    maintainer_email='holger.schwegmann@uni-luebeck.de',
    description='Rescue Robotics Quadrotor Assignment',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_detector_status = rescue_quadrotor_exercise.human_detector_status:main',
            'pcontroller_base = rescue_quadrotor_exercise.pcontroller_base:main',
            'QuadrotorGoalPublisher = rescue_quadrotor_exercise.QuadrotorGoalPublisher:main'
        ],
    },
)
