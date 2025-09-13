from setuptools import find_packages, setup

package_name = 'point_cloud_pose_estimator'

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
    maintainer='waseem',
    maintainer_email='waseem@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'point_estimator_node = point_cloud_pose_estimator.point_estimator_node:main',
            'football_detect = point_cloud_pose_estimator.football_detect:main',
        ],
    },
)
