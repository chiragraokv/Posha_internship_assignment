from setuptools import setup

package_name = 'piper_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name] if False else []),  # optional
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/posha_sim.launch.py']),
        ('share/' + package_name + '/urdf', ['urdf/piper_description.urdf']),
        ('share/' + package_name + '/objects', [
            'objects/model.config',
            'objects/model.sdf',
            'objects/model.stl'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laserhammer',
    maintainer_email='laserhammer@example.com',
    description='POSHA Simulation Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)