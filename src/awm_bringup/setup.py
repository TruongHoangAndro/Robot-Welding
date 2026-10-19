import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'awm_bringup'


def tree(src):
    """data_files entries for every file under src (keeps the directory layout)."""
    out = {}
    for path in glob(os.path.join(src, '**', '*'), recursive=True):
        if os.path.isfile(path):
            out.setdefault(os.path.join('share', package_name, os.path.dirname(path)), []).append(path)
    return list(out.items())


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ] + tree('repo_template'),
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Truong Hoang',
    maintainer_email='truongxhoang@apillis.com',
    description='APLSMN601 launch files, D16 configuration, repository template.',
    license='Proprietary',
    tests_require=['pytest'],
)
