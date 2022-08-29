from setuptools import setup

package_name = 'neato_node2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['meshes/main_body.stl']),
        ('share/' + package_name, ['urdf/neato.urdf']),
        ('share/' + package_name, ['launch/robot_state_publisher.py']),
        ('share/' + package_name, ['launch/bringup_minimal.py']),
        ('share/' + package_name, ['launch/bringup.py']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paul Ruvolo',
    maintainer_email='pruvolo@olin.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'neato_node = neato_node2.neato_node:main',
            'setup_udp_stream = neato_node2.setup_udp_stream:main'
        ],
    },
)
