from setuptools import find_packages, setup

package_name = 'my_robot_commander_py'

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
    maintainer='Mudit Todi',
    maintainer_email='your-email@example.com',
    description='Python commander and monitoring tools for MRAC robot arm',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_moveit        = my_robot_commander_py.test_moveit:main',
            'mrac_trainer       = my_robot_commander_py.mrac_trainer:main',
            'mrac_monitor       = my_robot_commander_py.mrac_monitor:main',
            'mrac_system_test   = my_robot_commander_py.mrac_system_test:main',
            'mrac_integration_test = my_robot_commander_py.mrac_integration_test:main',
            'diagnose_mrac      = my_robot_commander_py.diagnose_mrac:main',
        ],
    },
)
