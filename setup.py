from setuptools import find_packages, setup

package_name = 'bb8_navigation'

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
    maintainer='Tzu-Tung Chen',
    maintainer_email='tchen604@gatech.edu',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'navigate=bb8_navigation.navigate:main',
        	'test=bb8_navigation.test:main'
        ],
    },
)
