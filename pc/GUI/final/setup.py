from setuptools import find_packages, setup

package_name = 'final'

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
    maintainer='computer520',
    maintainer_email='computer520@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_first_node = final.my_first_node:main',
            'my_subscriber = final.my_subscriber:main',
            'my_publisher = final.my_publisher:main',
            'alarm2 = final.alarm2:main'
        ],
    },
)
