from setuptools import find_packages, setup

package_name = 'vlmaps_dataset_recorder'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='robotica-cidis',
    maintainer_email='robotica-cidis@localhost',
    description='ROS 2 recorder for VLMaps RGB-D datasets',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vlmaps_dataset_recorder = vlmaps_dataset_recorder.recorder_node:main',
            'mapping_pose_runner = vlmaps_dataset_recorder.mapping_pose_runner:main',
        ],
    },
)
