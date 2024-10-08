from setuptools import find_packages, setup

package_name = 'visualization_inputs'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='farquham',
    maintainer_email='mfarquharson1064@gmail.com',
    description='handles all the visualization functionality also takes inputs from the user and forwards those',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vis = visualization_inputs.vis:main',
        ],
    },
)
