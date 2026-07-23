from setuptools import find_packages, setup

package_name = 'isaac_rover'

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
    maintainer='john',
    maintainer_email='jumasaet@espol.edu.ec',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'teleop_jackal = isaac_rover.teleop_jackal:main',
        'save_rgb_depth = isaac_rover.save_rgb_depth:main',
        'save_segmentation = isaac_rover.save_segmentation:main',
        'save_pcl = isaac_rover.save_pcl:main',
        'save_all = isaac_rover.save_all:main',
        'movement = isaac_rover.movement:main'
        ],
    },
)
