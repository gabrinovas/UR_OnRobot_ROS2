from setuptools import setup, find_packages

package_name = 'ur_onrobot_moveit_config'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gabriel Novas',
    maintainer_email='gabriel.novas@aimen.es',
    description='MoveIt2 configuration for UR robots with OnRobot grippers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)