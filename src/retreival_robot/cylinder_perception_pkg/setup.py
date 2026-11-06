from setuptools import setup

package_name = 'cylinder_perception'
setup(name=package_name, version='0.0.1', packages=[package_name],
      data_files=[('share/ament_index/resource_index/packages', ['resource/' + package_name]),
                  ('share/' + package_name, ['package.xml'])],
      install_requires=['setuptools'], zip_safe=True,
      maintainer='robotics', maintainer_email='robotics@example.com',
      description='Point cloud cylinder detector for Gazebo RGB-D camera',
      license='Apache-2.0',
      entry_points={'console_scripts': ['cylinder_detector = cylinder_perception.cylinder_detector:main']})
