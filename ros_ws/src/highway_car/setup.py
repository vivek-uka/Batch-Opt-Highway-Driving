from setuptools import setup

package_name = 'highway_car'

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
    maintainer='vivek',
    maintainer_email='vivekadajania@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                # 'highway_node = highway_car.highway_car:main',
                'highway_node2 = highway_car.highway_car2:main'
        ],
    },
)
