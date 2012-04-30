import os.path

import yaml

import roslib
roslib.load_manifest('rosbag')
import rosbag.bag

def camera_calibration_yaml_to_bagfile(yamlpath, extrinsics, bagpath):
    pass

def camera_calibration_yaml_to_radfile(yamlpath, bagpath):
    with open(yamlpath,'r') as yf:
        y = yaml.load(yf)
        K = y['camera_matrix']['data']
        dist = y['distortion_coefficients']['data']
        with open(bagpath,'w') as bf:
            for row in range(3):
                for col in range(3):
                    i = col + row*3
                    bf.write("K%d%d = %f\n" % (row+1,col+1,K[i]))
            bf.write("\n")

            #XXX: in opencv (i.e. as is the yaml file), the coeffs are k1,k2,p1,p2,k3
            #and in MultiCamSelfCal I am not sure what kc3 and kc4 are... so just do the
            #first two, the most important ones (the radial dist. coeffs k1=kc1, k2=kc2)
            for i in range(2):
                bf.write("kc%d = %f\n" % (i+1,dist[i]))
            bf.write("kc3 = 0.0\n")
            bf.write("kc4 = 0.0\n")
            bf.write("\n")
            

