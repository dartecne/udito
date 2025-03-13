from setuptools import find_packages, setup

package_name = 'respeaker_ros'

setup(
    name=package_name,
    version='0.0.0',
#    packages=find_packages(exclude=['test']),
    packages=find_packages(),
#    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='udito',
    maintainer_email='dartecne@gmail.com',
<<<<<<< HEAD
    description='Audio',
=======
    description='Manejador de la cabeza de UDITO',
>>>>>>> d75813b06134359daeaea89edeabe871241fbad3
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'respeaker_node = respeaker_ros.respeaker_node:main'
        ],
    },
)
