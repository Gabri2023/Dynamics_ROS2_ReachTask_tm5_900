
import os 
from glob import glob
from setuptools import setup

package_name = 'my_doosan_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),    
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
        (os.path.join('share', package_name, 'worlds'), glob(os.path.join('worlds', '*.world'))),           
        (os.path.join('share', package_name, 'description','desc_tm900', 'urdf'), glob(os.path.join('description','desc_tm900','urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'description','desc_tm900', 'urdf'), glob(os.path.join('description','desc_tm900','urdf', '*.xacro'))),
        (os.path.join('share', package_name, 'description','desc_tm900', 'meshes','tm5_900','collision'), glob(os.path.join('description','desc_tm900','meshes','tm5_900','collision', '*.stl'))),
        (os.path.join('share', package_name, 'description','desc_tm900', 'meshes','tm5_900','visual'), glob(os.path.join('description','desc_tm900','meshes','tm5_900','visual', '*.stl'))),
        (os.path.join('share', package_name, 'description','desc_tm900', 'meshes','tm5_900','visual'), glob(os.path.join('description','desc_tm900','meshes','tm5_900','visual', '*.mtl'))),
        (os.path.join('share', package_name, 'description','desc_tm900', 'meshes','tm5_900','visual'), glob(os.path.join('description','desc_tm900','meshes','tm5_900','visual', '*.obj'))),
               
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='david',
    maintainer_email='david@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'trajectory_points_topic = my_doosan_pkg.joint_points_topic:main',
                'trajectory_points_act_server = my_doosan_pkg.joint_points_act_service:main',
        ],
    },
)
