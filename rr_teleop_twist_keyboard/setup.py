from setuptools import setup

package_name = 'rr_teleop_twist_keyboard'

setup(
    name=package_name,
    version='2.4.0',
    packages=[],
    py_modules=[
        'rr_teleop_twist_keyboard'
    ],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chris Lalancette',
    maintainer_email='clalancette@openrobotics.org',
    author='Graylin Trevor Jay, Austin Hendrix',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A robot-agnostic teleoperation node to convert keyboard'
                'commands to Twist messages, but for RinkRover.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rr_teleop_twist_keyboard = rr_teleop_twist_keyboard:main'
        ],
    },
)
