from setuptools import find_packages, setup

package_name = 'awm_cad'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Truong Hoang',
    maintainer_email='truongxhoang@apillis.com',
    description='APLSMN601 M1 CAD import (A11-A14): ROS-side wrapper of the awm_geometry CLI.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'import_step = awm_cad.cli:main',
        ],
    },
)
