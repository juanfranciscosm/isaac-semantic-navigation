from setuptools import find_packages, setup

package_name = 'semantic_navigator'

setup(
    name=package_name,
    version='0.1.0',
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
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica-cidis',
    maintainer_email='robotica-cidis@localhost',
    description=(
        'Envía metas semánticas al servidor NavigateToPose de Nav2.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            (
                'semantic_navigator_node = '
                'semantic_navigator.semantic_navigator_node:main'
            ),
        ],
    },
)