# setup.py v2.010 2026-01-25
# [이번 버전에서 수정된 사항]
# - (변수수정) config/*.yaml을 share/<pkg>/config로 설치하도록 data_files에 추가
# - (정리) 테스트용 basic_bio_action_server 엔트리 제거

import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bio_transport'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 등록
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        # config yaml 설치 등록
        #(os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gom',
    maintainer_email='gom@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'bio_main = bio_transport.main_integrated:main',      # 메인 오케스트레이터
            'bio_sub = bio_transport.rack_transport_action:main',  # 하위 로봇 제어
            'bio_ui = bio_transport.ui_integrated:main',
        ],
    },
)
