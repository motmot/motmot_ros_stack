#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
import rospy

roslib.load_manifest('motmot_ros_utils')
import rosutils.io as io
import rosutils.formats as formats

import xmlrpclib

def test_io():
    print io.decode_url('~/Documents')
    print io.decode_url('/tmp')
    print io.decode_url('file:///home/strawlab')
    print io.decode_url('package://vros_display/foo/bar')
    print io.decode_url('package://vros_display/calib/${NODE_NAME}.xml')
    print io.decode_url('${ROS_HOME}/foo')

def test_decode_file():
    PATH = '/testmotmotrosutils'
    DATA = '123'
    b = xmlrpclib.Binary(data=DATA)
    rospy.set_param(PATH,b)

    a = io.decode_file_from_parameter_server(PATH)
    b = io.decode_url('parameter://'+PATH)
    assert a == b

    d = open(a).read()
    assert d == DATA
    

def test_formats():
    
    urlyaml = io.decode_url('package://motmot_ros_utils/tests/Basler_21029383.yaml')
    urlrad = io.decode_url('package://motmot_ros_utils/tests/Basler_21029383.rad')

    rad = '/tmp/test.rad'
    formats.camera_calibration_yaml_to_radfile(urlyaml, rad)
    assert open(urlrad,'r').read() == open(rad,'r').read()
    
if __name__ == "__main__":
    rospy.init_node('test', anonymous=True)
    test_io()
    test_formats()
    test_decode_file()
