from setuptools import find_packages, setup

package_name = 'tello_parcours'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='schoe',
    maintainer_email='schoe@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'parcours_runner = tello_parcours.parcours_runner:main',
            'parcours_runner_tello = tello_parcours.parcours_runner_tello:main',
            'single_hoop_test = tello_parcours.single_hoop_test:main',
            'autopilot = tello_parcours.autopilot:main',
            'camera_viewer = tello_parcours.camera_viewer:main',
            'hoepel_jager = tello_parcours.hoepel_jager:main',
            'hsv_tuner_tello = tello_parcours.hsv_tuner_tello:main',
        ],
    },
)
