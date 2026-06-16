from setuptools import find_packages, setup

package_name = 'drone_traffic_client'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='JacopoPan',
    maintainer_email='jacopo.pan@gmail.com',
    description='Drone traffic client',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dtc_client = drone_traffic_client.dtc_client_node:main',
        ],
    },
)
