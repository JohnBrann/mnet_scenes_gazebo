import os

from setuptools import find_packages, setup
from glob import glob

def package_files(root_dir):
    paths = []
    for path, _, filenames in os.walk(root_dir):
        for f in filenames:
            full = os.path.join(path, f)
            rel = os.path.relpath(full, root_dir)
            paths.append((os.path.join('share', package_name, 'models', os.path.dirname(rel)), [full]))
    return paths

package_name = 'create_object_scenes'

model_files = package_files('models')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/scenes', glob('scenes/*.npz')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        *model_files,
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csrobot',
    maintainer_email='john.brann4315@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
