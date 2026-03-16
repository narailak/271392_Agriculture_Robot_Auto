from setuptools import find_packages, setup

package_name = 'agri_control'

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
    maintainer='aorus-ubun',
    maintainer_email='oatdev54@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_mapper = agri_control.joy_mapper:main',
            'mode_mux = agri_control.mode_mux:main',
            'teleop_keyboard = agri_control.teleop_keyboard:main',
            'pid_controller_node = agri_control.pid_controller_node:main',
            'odometry_node = agri_control.odometry_node:main',
            'final_april_tag_node = agri_control.final_april_tag_node:main',
        ],
    },
)