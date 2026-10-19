from setuptools import find_packages, setup

package_name = 'awm_program'

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
    description='APLSMN601 M6 program generator (A61-A64).',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate = awm_program.node:main',
        ],
    },
)
