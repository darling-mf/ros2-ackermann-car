from setuptools import find_packages, setup

package_name = 'r5_serial'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/docs', ['docs/S5_SERIAL_BRIDGE.md']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mjf',
    maintainer_email='darling@ros.com',
    description='X-Protocol serial bridge + virtual STM32 for TARKBOT R5 (S5)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtual_stm32 = r5_serial.virtual_stm32:main',
            'serial_bridge = r5_serial.serial_bridge:main',
        ],
    },
)
