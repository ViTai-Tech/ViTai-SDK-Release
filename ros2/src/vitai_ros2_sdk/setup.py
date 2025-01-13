from setuptools import find_packages, setup

package_name = 'vitai_ros2_sdk'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['loguru', 'setuptools'],
    zip_safe=True,
    maintainer='sun',
    maintainer_email='1247727512@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "vt_publisher_node = vitai_ros2_sdk.vt_publisher_node:main",
            "vt_subscriber_node = vitai_ros2_sdk.vt_subscriber_node:main"
        ],
    },
)
