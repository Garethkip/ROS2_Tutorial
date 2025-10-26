from setuptools import find_packages, setup

package_name = 'my_py_pkg'

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
            "my_node_execute = my_py_pkg.my_first_node:main",#<executable_name> = <package_name>.<file_name>:<function_name>
            "basic_node = my_py_pkg.basic_python_node:main", 
            "shifter = my_py_pkg.shifter_node:main",
            "basic_pub = my_py_pkg.basic_publisher:main",
            "basic_sub = my_py_pkg.basic_subscriber:main",
            "wave_pub = my_py_pkg.wave_publisher:main",
            "wave_sub = my_py_pkg.wave_subscriber:main"
        ],
    },
)
