from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'example_acados_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tpoignonec',
    maintainer_email='tpoignonec@unistra.fr',
    description='Bringup package for the Acados example(s).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publish_dummy_reference = example_acados_bringup.publish_dummy_reference:main'
        ],
    },
)
