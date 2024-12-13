from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'youbot_description'

# Iterate through all the files and subdirectories
# to build the data files array
def generate_data_files(share_path, dir):
    data_files = []
    
    for path, _, files in os.walk(dir):
        list_entry = (share_path + path, [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdfs', ['urdfs/youbot.urdf'])
    ]  + generate_data_files('share/' + package_name + '/', 'meshes'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eddie Edwards',
    maintainer_email='eddie.edwards@ucl.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
