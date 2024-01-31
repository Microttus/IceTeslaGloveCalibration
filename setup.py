from setuptools import find_packages, setup

package_name = 'ice_tesla_glove_calibration'

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
    maintainer='biomech',
    maintainer_email='martino@uia.no',
    description='Package containg code for ccalibration of IceCube Tesla Haptic Glove equipment',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ice_tesla_haptic_calibration = ice_tesla_glove_calibration.ice_tesla_glove_cal:main'
        ],
    },
)
