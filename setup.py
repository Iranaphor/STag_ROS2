from setuptools import setup
from glob import glob
import os

package_name = 'stag_ros2'
pkg = package_name

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{pkg}']),
        (f'share/{pkg}', ['package.xml']),
        (f"share/{package_name}/launch", glob(os.path.join('launch', '*launch.[pxy][yml]*'))),
        (f"share/{package_name}/config", glob(os.path.join('config', '*'))),
        (f"share/{package_name}/bash", glob(os.path.join('bash', '*.sh')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='james',
    maintainer_email='primordia@live.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'processor.py = {pkg}.processor:main',
            f'calibrator.py = {pkg}.calibrator:main',
            f'renderer.py = {pkg}.renderer:main',
            f'mqtt_forwarder.py = {pkg}.mqtt_forwarder:main',
            f'mqtt_receiver.py = {pkg}.mqtt_receiver:main',
        ],
    },
)
