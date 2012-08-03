import os.path
import numpy as np

import yaml

def camera_calibration_yaml_to_radfile(yamlpath, radpath, lossy_ok=False):
    with open(yamlpath,'r') as yf:
        y = yaml.load(yf)

        K = y['camera_matrix']['data']
        Knp = np.array(K)
        Knp.shape = y['camera_matrix']['rows'],y['camera_matrix']['cols']

        P = y['projection_matrix']['data']
        P = np.array(P)
        P.shape = y['projection_matrix']['rows'],y['projection_matrix']['cols']
        assert np.allclose(P[:,3], np.zeros((3,)))

        if not lossy_ok and not np.allclose(Knp,P):
            raise ValueError('cannot do lossless conversion to MultiCamSelfCal '
                             '.rad file (matrices differ)')
        assert y['distortion_model']=='plumb_bob'
        dist = y['distortion_coefficients']['data']
        assert len(dist)==5
        if dist[4]!=0.0:
            raise ValueError('cannot do lossless conversion to MultiCamSelfCal '
                             '.rad file (dist. coeff. k3 was %f)' % dist[4])
        with open(radpath,'w') as bf:
            for row in range(3):
                for col in range(3):
                    i = col + row*3
                    bf.write("K%d%d = %f\n" % (row+1,col+1,K[i]))
            bf.write("\n")

            for i in range(4):
                bf.write("kc%d = %f\n" % (i+1,dist[i]))
            bf.write("\n")
