from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup(**generate_distutils_setup(packages=['ft_rqt_gui'], package_dir={'': 'src'}))
