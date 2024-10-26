from setuptools import setup

from glob import glob
import os

package_name = 'mybot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/**')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/**')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/**')),
        (os.path.join('share', package_name, 'world'), glob('world/**')),
        (os.path.join('share', package_name, 'weights'), glob('weights/**')),
    ],
    install_requires=['setuptools', 'opencv-python', 'cv_bridge'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='leo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rotate_wheel= mybot_description.rotate_wheel:main",
            'image_detection = mybot_description.image_detection:main',
            'move_arm = mybot_description.move_arm:main',
            'move_claw = mybot_description.move_claw:main',
            'moveit_move_arm = mybot_description.moveit_move_arm:main',
            'audio_classify = mybot_description.audio_classify:main',
            'navigate = mybot_description.navigate:main',
        ],
    },
)

