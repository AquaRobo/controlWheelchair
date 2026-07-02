from setuptools import find_packages, setup
from glob import glob

package_name = 'cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/models', glob('models/*')),
    ],
    install_requires=['setuptools', 
                      'utils'],
    zip_safe=True,
    maintainer='mansour',
    maintainer_email='ahmedmonsour5@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'object_detection_node = cv.nodes.ObjectDetectionNode:main',
            'room_identifier_node = cv.nodes.RoomIdentifierNode:main',
            'speech_recognizer_node = cv.nodes.SpeechRecognizerNode:main',
            'camera_visualizer_test_node = cv.tests.CameraVisualizerTestNode:main',
        ],
    },
)
