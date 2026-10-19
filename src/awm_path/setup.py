from setuptools import find_packages, setup

package_name = 'awm_path'

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
    description='APLSMN601 M4 path planner (A41-A45).',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plan_path = awm_path.node:main',
        ],
    },
)
