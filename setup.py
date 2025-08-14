from setuptools import setup

package_name = 'dashboard_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cyclone RoboSub UC Davis',
    maintainer_email='crs.aggies@gmail.com',
    description='Dashboard Package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_publisher = dashboard_project.position_publisher:main',
        ],
    },
)