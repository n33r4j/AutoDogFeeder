from setuptools import find_packages, setup

package_name = 'adf_prototype'

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
    maintainer='vscode',
    maintainer_email='n33r4j@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_console = adf_prototype.MainConsole:main',
            'camera = adf_prototype.Camera:main',
            'feeder = adf_prototype.Feeder:main',
            'detector = adf_prototype.Detector:main',
        ],
    },
)
