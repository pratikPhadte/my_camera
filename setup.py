from setuptools import find_packages, setup

package_name = 'my_camera'

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
    maintainer='pratik',
    maintainer_email='pratikphadte19@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_node = my_camera.webcam_node:main',
            'usbcam_node = my_camera.usbcam_node:main',
            'ip_stream_node = my_camera.ip_stream_node:main',      
        ],
    },
)
