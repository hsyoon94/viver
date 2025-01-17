from setuptools import find_packages, setup

package_name = 'ocams_processor'

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
            'stereo_image_processor = ocams_processor.stereo_image_processor:main',
            'imu_processor = ocams_processor.imu_processor:main',
        ],
    },
)
