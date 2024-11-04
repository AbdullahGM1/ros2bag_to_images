from setuptools import setup

package_name = 'ros2bag_to_image'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AbdullahGM',
    maintainer_email='agm.musalami@gmail.com',
    description='A package to extract images from a ROS2 bag file.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'extract_images = bag_image_extractor.extract_images:main',
        ],
    },
)

