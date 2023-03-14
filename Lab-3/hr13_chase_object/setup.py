from setuptools import setup
import glob

package_name = 'hr13_chase_object'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ('share/' +  package_name + '/launch/', ['chase_object.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='himanshu',
    maintainer_email='hvairagade3@gatech.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_object = ' + package_name + '.detect_object:main',
            'chase_object = ' + package_name + '.chase_object:main',
            'get_object_range = ' + package_name + '.get_object_range:main',
        ],
    },
)
