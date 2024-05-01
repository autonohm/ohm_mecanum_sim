from setuptools import find_packages, setup

package_name = 'ohm_mecanum_sim'

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
    maintainer='Stefan May',
    maintainer_email='stefan.may@th-nuernberg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ohm_mecanum_sim_node = ohm_mecanum_sim.ohm_mecanum_sim_node:main'
        ],
    },
)
