from setuptools import setup

package_name = 'astable_invention'
submodules = package_name + "/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jakub',
    maintainer_email='jakub@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
			'listener = astable_invention.listener:main', # this is mock, for initial testing
            'publisher = astable_invention.publisher:main', # this is mock, for initial testing
            'main = astable_invention.main:main'
        ],
    },
)
