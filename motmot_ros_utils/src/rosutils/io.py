import roslib
import roslib.packages
import roslib.rosenv

roslib.load_manifest('rospy')
import rospy.names

import os.path
import string

def decode_url(url, required=False):
    """
    Example URL syntax:
        file:///full/path/to/local/file.yaml
        file:///full/path/to/videre/file.ini
        package://camera_info_manager/tests/test_calibration.yaml
        package://ros_package_name/calibrations/camera3.yaml
        /full/path
        ~/home/path

    The file: URL specifies a full path name in the local system. The package: URL is handled
    the same as file:, except the path name is resolved relative to the location of the named ROS
    package, which must be reachable via $ROS_PACKAGE_PATH.

    The following variables can be replaced

        ${NODE_NAME} resolved to the current node name.
        ${ROS_HOME} resolved to the $ROS_HOME environment variable if defined, ~/.ros if not.

    XXXX: Use the ros resource_retriever class here one day, when it has python bindings and
          variable substitution
    """
    if url.startswith('~'):
        url = os.path.expanduser(url)
    elif url.startswith('file://'):
        url = url[7:]
    elif url.startswith('package://'):
        package,fragment = url[10:].split('/',1)
        url = os.path.join(roslib.packages.get_pkg_dir(package,required=False),fragment)

    nodename = rospy.names.get_name()
    if nodename:
        #rospy returns rully resolved name, so remove first char (~,/) and
        #return only the first fragment
        nodename = nodename[1:].split(roslib.names.SEP)[-1]
    else:
        nodename = ''

    url = string.Template(url).safe_substitute(
                                NODE_NAME=nodename,
                                ROS_HOME=roslib.rosenv.get_ros_home())

    url = os.path.abspath(url)
    if required and not os.path.exists(url):
        raise Exception("resource not found")

    return url

