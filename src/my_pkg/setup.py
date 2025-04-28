from setuptools import setup

package_name = 'my_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kirupa',
    maintainer_email='kirupa@todo.todo',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = my_pkg.publisher_node:main',
            'listener = my_pkg.subscriber_node:main',
            'service = my_pkg.service_node:main',
            'client = my_pkg.client_node:main',
            'AEB = my_pkg.automatic_breaking:main',
        ],
    },
)
