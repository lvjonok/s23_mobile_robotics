from setuptools import setup

package_name = 's23_mobile_robotics'

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
    maintainer='leo',
    maintainer_email='kozlov.l.a@mail.ru',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hw1 = s23_mobile_robotics.homework1:main',
            'hw2 = s23_mobile_robotics.homework2:main',
            'hw3 = s23_mobile_robotics.homework3:main',
            'hw4 = s23_mobile_robotics.homework4:main'
        ],
    },
)
