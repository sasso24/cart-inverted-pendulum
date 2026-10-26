from glob import glob

from setuptools import setup

PACCHETTO = 'pendolo_mocap'

setup(
    name=PACCHETTO,
    version='0.1.0',
    packages=[PACCHETTO],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{PACCHETTO}']),
        (f'share/{PACCHETTO}', ['package.xml']),
        (f'share/{PACCHETTO}/config', glob('config/*.yaml')),
        (f'share/{PACCHETTO}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    description='Angolo del pendolo dal tracciamento alla Nucleo via seriale',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ponte_mocap = pendolo_mocap.ponte_mocap:main',
            'nodo_test = pendolo_mocap.nodo_test:main',
        ],
    },
)
