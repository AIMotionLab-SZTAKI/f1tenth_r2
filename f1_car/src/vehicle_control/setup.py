from setuptools import setup
from glob import glob
import os

package_name = 'vehicle_control'
packages = [package_name, package_name + "/" + "submodules"]
setup(
    name=package_name,
    version='0.0.0',
    packages=packages,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/'+package_name , glob('launch/*launch.py')),
        (os.path.join('share', package_name), glob('config/param.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bodlaire',
    maintainer_email='bodabenedek03@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control = vehicle_control.control:main'
        ],
    },
)