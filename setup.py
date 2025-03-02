from setuptools import find_packages, setup

package_name = 'simplevis'

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
    maintainer='MaxAve',
    maintainer_email='maxave909@gmail.com',
    description='Simple visualization for sensor messages',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtlebot = simplevis.turtlebot:main',
        ],
    },
)
