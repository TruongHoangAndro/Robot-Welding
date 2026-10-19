from setuptools import find_packages, setup

package_name = 'awm_process'

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
    description='APLSMN601 M3 process parameters (A31-A33): WPS library, process selection, process.json.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'assign = awm_process.cli:main',
        ],
    },
)
