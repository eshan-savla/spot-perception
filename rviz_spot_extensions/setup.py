from setuptools import find_packages, setup

package_name = 'rviz_spot_extensions'

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
    maintainer='TurboEshan',
    maintainer_email='TurboEshan@h-ka.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigate_to = rviz_spot_extensions.nav2_action_client:main'
        ],
    },
)
