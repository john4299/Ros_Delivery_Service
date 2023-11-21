from setuptools import find_packages, setup

package_name = 'depth_example'

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
    maintainer='noma',
    maintainer_email='noma@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'detect_node = depth_example.detect_node:main',
        'final_node = depth_example.final_node:main',
        'final_RS_node = depth_example.final_RS_node:main',
        'test_node = depth_example.test_node:main',
        'mail_node = depth_example.mail_node:main',
        ],
    },
)
