from setuptools import find_packages, setup
import os

package_name = 'adam_localize'

def get_package_data_files(directory, target_base):
    """Get all files in directory recursively, maintaining directory structure"""
    data_files = []
    
    for root, dirs, files in os.walk(directory):
        if files:
            # Calculate target directory
            rel_path = os.path.relpath(root, directory)
            if rel_path == '.':
                target_dir = target_base
            else:
                target_dir = os.path.join(target_base, rel_path).replace('\\', '/')
            
            # Get all files with their full paths
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((target_dir, file_list))
    
    return data_files

# Start with the required basic files
data_files_list = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# Add files from launch and config directories
if os.path.exists('launch'):
    data_files_list.extend(get_package_data_files('launch', f'share/{package_name}/launch'))

if os.path.exists('config'):
    data_files_list.extend(get_package_data_files('config', f'share/{package_name}/config'))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files_list,  # Use data_files, not data_files_list
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lynx',
    maintainer_email='lynx.thoi@emagegroup.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'adam_localize = adam_localize.adam_localize:main', 
        ],
    },
)