from setuptools import find_packages, setup

package_name = 'mower_binary_grid'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/binary_grid.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jangwoo',
    maintainer_email='jangwoo@example.com',
    description='Fuse AFR semantics and aligned D455 depth into a 20 cm binary local grid',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'binary_grid_node = mower_binary_grid.binary_grid_node:main',
        ],
    },
)

