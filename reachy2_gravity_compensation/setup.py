from setuptools import find_packages, setup

package_name = 'reachy2_gravity_compensation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pin==2.7.0'],
    zip_safe=True,
    maintainer='reachy',
    maintainer_email='reachy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        "console_scripts": [
            "gravity_compensator = reachy2_gravity_compensation.gravity_compensator:main",
        ],
    },
)
