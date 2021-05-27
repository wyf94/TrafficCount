## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

#from setuptools import setup
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    version='0.0.0',
    scripts=['node/image_publisher_node.py', 'node/traffic_count_node.py'],
    packages=["traffic_count"], 
    package_dir={"": "src"},
    requires=['std_msgs', 'rospy', 'sensor_msgs']
)

setup(**setup_args)
