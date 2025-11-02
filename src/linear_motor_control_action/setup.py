from setuptools import find_packages, setup

package_name = 'linear_motor_control_action'

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
    maintainer='g',
    maintainer_email='garethkip@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "action_server = linear_motor_control_action.action_server:main",
            "action_client = linear_motor_control_action.action_client:main",
        ],
    },
)
