from setuptools import find_packages, setup

package_name = 'tello_joy'

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
    maintainer='ifcosta',
    maintainer_email='ifcosta@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tello_joy = tello_joy.tello_joy:main',
            'filter_joy = tello_joy.filter_joy:main'
        ],
    },
)
