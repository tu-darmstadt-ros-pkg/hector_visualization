from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['hector_rqt_joint_control'],
    scripts=['scripts/joint_control', 'scripts/joint_control_qt'],
    package_dir={'': 'src'}
)

setup(**d)
