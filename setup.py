import os
from glob import glob
from setuptools import setup

PACKAGE_NAME = 'squarbo_gazebo'

setup(
    name=PACKAGE_NAME,
    version='1.0.0',
    package_dir={'': 'src'},
    data_files=[
        (os.path.join('share', PACKAGE_NAME), glob('launch/*.launch.py')),
        (os.path.join('share', PACKAGE_NAME), glob('world/*.world'))
    ],
    packages=[PACKAGE_NAME],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spawner_squarbo = squarbo_gazebo.spawner_squarbo:main',
        ],
    },
)
