from setuptools import find_packages, setup

package_name = 'head_package'

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
    maintainer='udito',
    maintainer_email='dartecne@gmail.com',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'head_node = head_package.head_node:main',
            'talker = head_package.publisher_member_function:main',
            'listener = head_package.subscriber_member_function:main'
        ],
    },
)
