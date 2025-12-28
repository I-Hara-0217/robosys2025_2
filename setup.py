from setuptools import setup
import os
from glob import glob

package_name = 'robosys2025_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), ['words.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ibuki Hara',
    maintainer_email='i.hara.git@gmail.com',
    description='ロボットシステム学課題2',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'quotes_publisher = robosys2025_2.quotes_publisher:main',
            'quotes_subscriber = robosys2025_2.quotes_subscriber:main',
        ],
    },
)
