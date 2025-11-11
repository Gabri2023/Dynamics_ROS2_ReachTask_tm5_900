from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_environment_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    
    
        # aggiungi qui launch, config o description se servono
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabri',
    maintainer_email='gabri@example.com',
    description='Environment package for TM5-900 robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run_environment_2 = my_environment_pkg.run_environment_2:main',
            'test_agent = my_environment_pkg.test_agent:main',
        ],
    },
)
