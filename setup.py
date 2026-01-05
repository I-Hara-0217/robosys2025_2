# SPDX-FileCopyrightText: 2025 Ibuki Hara
# SPDX-License-Identifier: BSD-3-Clause

from setuptools import setup

package_name = 'saying_sender'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['words.txt']),
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
            'quotes_publisher = saying_sender.quotes_publisher:main',
        ],
    },
)
