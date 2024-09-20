from setuptools import find_packages, setup

package_name = 'team59_object_follower'

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
    maintainer='ibrahim',
    maintainer_email='ibrahim@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "find_object = team59_object_follower.find_object:main",
            "view_image_test = team59_object_follower.view_image_raw:main",
            "view_image_raw2 = team59_object_follower.view_image_raw2:main",
            "rotate_robot = team59_object_follower.rotate_robot:main",
        ],
    },
)
