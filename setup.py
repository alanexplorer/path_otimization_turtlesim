from setuptools import setup
from setuptools import find_packages

package_name = 'path_optimization'

setup(
    name=package_name,
    version='0.0.0',
    # packages=[package_name],
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alan',
    maintainer_email='alan.pereira.dev@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'path_optimization_node = path_optimization.path_optimization_node:main',
            'turtlesim_test_node = path_optimization.turtlesim_test_node:main'
        ],
    },
)
