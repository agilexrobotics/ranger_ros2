from setuptools import find_packages, setup

package_name = 'ranger_mini_v3_sim_messenger'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/messenger.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shuaib Olanrewaju',
    maintainer_email='solanrewaju2020@fau.edu',
    description='Twist-to-controllers messenger for Ranger Mini v3 sim.',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_messenger = ranger_mini_v3_sim_messenger.sim_messenger:main',
        ],
    },
)
