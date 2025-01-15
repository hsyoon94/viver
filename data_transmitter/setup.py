from setuptools import find_packages, setup

package_name = 'data_transmitter'

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
    maintainer='viver1',
    maintainer_email='viver1@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'data_sender = data_transmitter.imu_processor:main',
            'stereo_image_processor = data_transmitter.stereo_image_processor:main',
            'image_sender = data_transmitter.image_sender:main',
            'imu_processor = data_transmitter.imu_processor:main',
        ],
    },
)
