from setuptools import setup

package_name = 'cf_ros'

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
    maintainer='Sohaib Bhatti',
    maintainer_email='sohaibbhatti192@gmail.com',
    description='ROS2 control of CrazyFlie through CLLib',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                            'cf_publisher = cf_ros.crazyflie:main'
        ],
    },
)
