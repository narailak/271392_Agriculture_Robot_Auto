from setuptools import find_packages, setup

package_name = 'cabbage_detection'

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
        'camera_publisher = cabbage_detection.camera_publisher_node:main',
        'apriltag_node = cabbage_detection.apriltag_node:main',
        'garden_yolo_node = cabbage_detection.garden_yolo:main',
        ],
    },
)
