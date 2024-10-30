from setuptools import find_packages, setup

package_name = 'vox_terminus'

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
    maintainer='athrva',
    maintainer_email='athrvakukarni11@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_pub=vox_terminus.speech_pub:main',
            'voxterminus_v1=vox_terminus.voxterminus_v1:main',
        ],
    },
)
