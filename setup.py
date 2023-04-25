# this file will reuse all information from your package.xml file
# and extend it with required dependencies for ROS python
# dont install this file directly, instead it will be found and
# installed automatically by adding the "catkin_python_setup()"-
# entry to your CMakeLists.txt project file
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['rxt_skills_alexa'], #same as name in package.xml!!
    #install_requires=['rospy', 'actionlib'], #external packages as dependencies are already defined in package.xml
    scripts=[
        'scripts/python/ALEXA_action_server_py'
    ] 
    #package_dir={'': 'src'}
)

setup(**d)
