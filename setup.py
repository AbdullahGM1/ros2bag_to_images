from setuptools import find_packages, setup

package_name = 'ros2bag_to_images'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AbdullahGM',
    maintainer_email='agm.musalami@gmail.com',
    description='Extract Images from Ros2 Humble Bag',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_image_extractor = ros2bag_to_images.bag_image_extractor:main'
        ],
    },
)
