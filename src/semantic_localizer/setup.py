from setuptools import find_packages, setup

package_name = 'semantic_localizer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='robotica-cidis',
    maintainer_email='robotica-cidis@localhost',
    description='Localizador semántico usando máscara CLIPSeg y profundidad.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'semantic_localizer_node = semantic_localizer.semantic_localizer_node:main',
        ],
    },
)