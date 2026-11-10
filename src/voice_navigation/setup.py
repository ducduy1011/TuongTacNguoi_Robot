from glob import glob
from setuptools import setup

setup(
    name='voice_navigation',
    version='0.1.0',
    packages=['voice_navigation'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/voice_navigation']),
        ('share/voice_navigation', ['package.xml']),
        ('share/voice_navigation/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    entry_points={'console_scripts': [
        'segment_map = voice_navigation.segment_map:main',
        'voice_nav = voice_navigation.voice_nav:main',
        'speech = voice_navigation.speech:main',
        'map_labels = voice_navigation.map_labels:main',
    ]},
)
