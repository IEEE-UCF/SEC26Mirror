from setuptools import setup
from glob import glob
import os

package_name = 'secbot_vision'

data_files = [
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'),
        glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'),
        glob('config/*.yaml')),
]

def package_files(directory, data_files, install_base):
    for (path, directories, filenames) in os.walk(directory):
        # Calculate install path relative to base directory
        rel_path = os.path.relpath(path, directory)
        if rel_path == '.':
            install_path = install_base
        else:
            install_path = os.path.join(install_base, rel_path)
            
        # Filter files to include
        files_to_install = []
        for filename in filenames:
            if filename.endswith(('.sdf', '.dae', '.png', '.mtl', '.stl', '.world', '.config', '.obj')):
                files_to_install.append(os.path.join(path, filename))
        
        if files_to_install:
            data_files.append((install_path, files_to_install))

# Add recursive world files
package_files('worlds', data_files, os.path.join('share', package_name, 'worlds'))

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deoxon',
    maintainer_email='your.email@example.com',
    description='Vision package for secbot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = secbot_vision.detector_node:main',
            'teleop_pid = secbot_vision.teleop_pid:main',
        ],
    },
)
