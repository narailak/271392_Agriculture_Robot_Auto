from setuptools import find_packages, setup

package_name = 'agri_vision'

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
            'apriltag_node = agri_vision.apriltag_node:main',
            'auto_front_plot = agri_vision.auto_front_plot:main',
            'measurement_node = agri_vision.measurement_node:main',
            'auto_front_seg = agri_vision.auto_front_seg:main', 
            'camera_publisher_node = agri_vision.camera_publisher_node:main',
        ],
    },
)
