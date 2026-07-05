from setuptools import find_packages, setup

package_name = 'px4_visualization'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/.gitkeep']),
        ('share/' + package_name + '/config', ['config/.gitkeep']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TanDatEmb',
    maintainer_email='TanDat.Emb@gmail.com',
    description='RViz, plotting and telemetry helpers for the PX4 navigation stack.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
