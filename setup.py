#!/usr/bin/env python

# from setuptools import setup

# package_name = 'frame_editor'

# setup(
#     name=package_name,
#     version='0.1.1',
#     package_dir={'': ''},
#     packages=[package_name],
#     data_files=[
#         ('share/ament_index/resource_index/packages',
#             ['resource/' + package_name]),
#         ('share/' + package_name, ['package.xml']),
#         ('share/' + package_name, ['plugin.xml'])
#     ],
#     install_requires=['setuptools'],
#     maintainer='your_name',
#     maintainer_email='your_email@example.com',
#     description='An editor for defining tf frames.',
#     entry_points={
#         'rqt_gui.plugins': [
#             'frame_editor = frame_editor.rqt_editor:FrameEditorGUI',
#         ],
#     },
# )


from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['frame_editor'],
    package_dir={'': 'frame_editor'}
)

setup(**d)
