from setuptools import find_packages, setup

package_name = 'pkg_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'cv_bridge',
        # Add other dependencies here if needed
    ],
    zip_safe=True,
    maintainer='othmene',
    maintainer_email='othmenderouichee@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_publisher = pkg_test.test_publisher:main',
            'test_subscriber = pkg_test.test_subscriber:main'
        ],
    },
)
