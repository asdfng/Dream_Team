from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['romi_soccer'],
    package_dir={'': 'bin' 'src'}
)

setup(**d)
