from setuptools import setup
from glob import glob
import os   
setup(
    name='second_exchange_station',
    version='0.0.1',
    packages=["second_exchange_station"],
    py_modules=[
        'main',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/second_exchange_station']),
        ('share/second_exchange_station_code', ['package.xml']),
        (os.path.join('share', 'second_exchange_station'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jlu',
)