from setuptools import setup
from glob import glob

package_name = 'sentry'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
  	('share/' + package_name+'/urdf/', glob('urdf/*')),
  	('share/' + package_name+'/rviz/', glob('rviz/*')),
  	('share/' + package_name+'/meshes/', glob('meshes/*')),
  	
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros-industrial',
    maintainer_email='2398772523@qq.com',
    description='sentry description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
