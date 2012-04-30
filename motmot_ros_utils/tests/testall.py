#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
import rospy

roslib.load_manifest('motmot_ros_utils')
import rosutils.io as io
import rosutils.formats as formats

def test_io():
    print io.decode_url('~/Documents')
    print io.decode_url('/tmp')
    print io.decode_url('file:///home/strawlab')
    print io.decode_url('package://vros_display/foo/bar')
    print io.decode_url('package://vros_display/calib/${NODE_NAME}.xml')
    print io.decode_url('${ROS_HOME}/foo')

def test_formats():
    url = io.decode_url('package://motmot_ros_utils/tests/Basler_21029383.yaml')
    rad = '/tmp/test.rad'

    formats.camera_calibration_yaml_to_radfile(url, rad)
    

if __name__ == "__main__":
    rospy.init_node('test', anonymous=True)
    #test_io()
    test_formats()
