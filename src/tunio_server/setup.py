from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tunio_server'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 관제 웹 정적 파일. web_node 가 share/tunio_server/web 에서 읽는다
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tunio-team',
    maintainer_email='team@example.com',
    description='tunio_server',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_manager = tunio_server.task_manager:main',
            'hmi_node = tunio_server.hmi_node:main',
            'web_node = tunio_server.web_node:main',
            'mock_state = tunio_server.mock_state:main',
            'result_store = tunio_server.result_store:main',
            'fake_hardware = tunio_server.fake_hardware:main',
        ],
    },
)
