from glob import glob

from setuptools import find_packages, setup

package_name = 'awm_verify'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/templates', glob('templates/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Truong Hoang',
    maintainer_email='truongxhoang@apillis.com',
    description='APLSMN601 M7 verification and release, offline chain runner.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'verify = awm_verify.node:verify_main',
            'record = awm_verify.records:main',
            'release = awm_verify.release:main',
            'run_chain = awm_verify.run_chain:main',
        ],
    },
)
