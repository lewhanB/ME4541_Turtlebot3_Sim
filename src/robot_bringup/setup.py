"""Install the bringup, Python Webots plugin, and local simulator assets."""

from pathlib import Path

from setuptools import find_packages, setup


package_name = 'robot_bringup'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml', 'LICENSE', 'NOTICE']),
]
for directory in ('launch', 'config', 'worlds', 'protos', 'scripts'):
    for path in sorted(Path(directory).rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            data_files.append((str(Path('share') / package_name / path.parent), [str(path)]))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Lewhan',
    maintainer_email='lewhan@example.com',
    description='TurtleBot3 Burger simulation on Windows Webots from WSL2.',
    license='Apache-2.0',
    tests_require=['pytest'],
    scripts=['scripts/check_bringup.py'],
)
