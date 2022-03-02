import os
from glob import glob
from setuptools import setup
package_name = 'mir_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
     ('share/ament_index/resource_index/packages',
      ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mir_bridge = mir_driver.mir_bridge:main',
            'fake_mir_joint_publisher = mir_driver.fake_mir_joint_publisher:main',
            'time_synchronizer = mir_driver.time_synchronizer:main',
            'tf_remove_child_frames = mir_driver.tf_remove_child_frames:main'
        ],
    },
)
