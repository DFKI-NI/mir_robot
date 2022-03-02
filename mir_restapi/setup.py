from setuptools import setup
import os
from glob import glob

package_name = 'mir_restapi'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                ('share/' + package_name, ['package.xml']),
                (os.path.join('share', package_name, 'launch'), glob('launch/*launch.py'))],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='niemann',
    maintainer_email='soenke.niemann@ipk.fraunhofer.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mir_restapi_server = mir_restapi.mir_restapi_server:main',
            'mir_restapi_sync_time = mir_restapi.mir_restapi_sync_time:main'
        ],
    },
)
