from setuptools import setup

package_name = 'first_package_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zeid',
    maintainer_email='zeidk@umd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'first_node_py = first_package_py.first_node_py:main',
            'publisher_py = first_package_py.publisher_py:main',
            'subscriber_py = first_package_py.subscriber_py:main'
        ],
    },
)
