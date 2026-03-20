from setuptools import find_packages, setup
from glob import glob

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*')),
    ],
    install_requires=['setuptools', 
                      'utils'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ahmedmonsour5@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_node = control.nodes.OdomNode:main',
            'navigation_node = control.nodes.NavigationNode:main',
            'room_pose_saver_node = control.nodes.RoomPoseSaverNode:main',
            'map_saver_node = control.nodes.MapSaverNode:main',
            'auto_nav_node = control.nodes.AutoNavNode:main',
            'speed_evaluator_test_node = control.test_nodes.SpeedEvaluatorTestNode:main',
            'steering_test_node = control.test_nodes.SteeringTestNode:main',
            'pwm_mapper_test_node = control.test_nodes.PWMMapperTestNode:main',
            'smoothing_test_node = control.test_nodes.SmoothingTestNode:main',
            'navigation_test_node = control.test_nodes.NavigationTestNode:main',
            'imu_test_node = control.test_nodes.IMUTestNode:main',
            'spi_first_test_node = control.test_nodes.SPIFirstTestNode:main',
            'spi_second_test_node = control.test_nodes.SPISecondTestNode:main',
            'spi_read_test_node = control.test_nodes.SPIReadTestNode:main',
        ],
    },
)
    