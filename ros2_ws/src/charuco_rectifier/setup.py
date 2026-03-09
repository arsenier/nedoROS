from setuptools import setup

package_name = 'charuco_rectifier'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nedoros',
    maintainer_email='nedoros@todo.todo',
    description='Rectify charuco board image using board pose',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'charuco_rectifier_node = charuco_rectifier.charuco_rectifier_node:main',
        ],
    },
)