from setuptools import find_packages, setup

package_name = 'awm_seam'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Truong Hoang',
    maintainer_email='truongxhoang@apillis.com',
    description='APLSMN601 M2 seam definition (A21, A22, A24): joint seams from picked edge ids, seams.json v2, seam_axis.json.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'seam = awm_seam.cli:main',
            'joint_seam = awm_seam.joint_cli:main',
        ],
    },
)
