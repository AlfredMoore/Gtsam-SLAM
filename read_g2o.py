import os
import numpy as np
from typing import Tuple

class G2O_data(object):

    def __init__(self, g2o_path: str) -> None:
        self.g2o_path = g2o_path
        self.poses = {}
        self.edges = {}

    def read_g2o_2d(self):
        with open(self.g2o_path, 'r') as f:
            for line in f.readlines():
                item = line.strip().split()
                # VERTEX_SE2
                if item[0] == 'VERTEX_SE2':
                    self.poses[ int( item[1] ) ] = {'type': item[0], 'ID': int(item[1]), 'CURRENT_ESTIMATE': [ float(i) for i in (item[2:]) ] }

                # EDGE_SE2
                elif item[0] == 'EDGE_SE2':
                    info_mat = np.array([[float(item[6]),float(item[7]),float(item[8])],
                                         [float(item[7]),float(item[9]),float(item[10])],
                                         [float(item[8]),float(item[10]),float(item[11])]], dtype=np.float64)

                    self.edges[ (int(item[1]), int(item[2])) ] = {'type': item[0], 'ID_SET': ( int( item[1] ), int( item[2] ) ),
                                                                'MEASUREMENT':[ float(i) for i in (item[3:6]) ], 
                                                                'INFO_MATRIX':info_mat
                                                                }
    
    def read_g2o_3d(self):
        with open(self.g2o_path,'r') as f:
            for line in f.readlines():
                item = line.strip().split()
                if item[0] == 'VERTEX_SE3:QUAT':
                    self.poses[ int( item[1] ) ] = {'type': item[0], 'ID': int(item[1]), 'CURRENT_ESTIMATE': [ float(i) for i in item[2:5]], 
                                                    'QUATERNION': [ float(i) for i in item[5:]]}

                elif item[0] == 'EDGE_SE3:QUAT':
                    info_mat = np.array([[float(item[10]),float(item[11]),float(item[12]),float(item[13]),float(item[14]),float(item[15])],
                                         [float(item[11]),float(item[16]),float(item[17]),float(item[18]),float(item[19]),float(item[20])],
                                         [float(item[12]),float(item[17]),float(item[21]),float(item[22]),float(item[23]),float(item[24])],
                                         [float(item[13]),float(item[18]),float(item[22]),float(item[25]),float(item[26]),float(item[27])],
                                         [float(item[14]),float(item[19]),float(item[23]),float(item[26]),float(item[28]),float(item[29])],
                                         [float(item[15]),float(item[20]),float(item[24]),float(item[27]),float(item[29]),float(item[30])]])
                    self.edges[ (int(item[1]), int(item[2])) ] = {'type': item[0], 'ID_SET': (int(item[1]), int(item[2])), 
                                                                  'MEASUREMENT': [ float(i) for i in item[3:6]], 
                                                                  'MEASREMENT_QUATERNION': [ float(i) for i in item[6:10]],
                                                                  'INFO_MATRIX': info_mat}



if __name__ == '__main__':
    g2o_2d = G2O_data(g2o_path="data/input_INTEL_g2o.g2o")
    g2o_2d.read_g2o_2d()
    # print(g2o_2d.poses[0])
    # print(g2o_2d.edges[(2,3)])

    g2o_3d = G2O_data(g2o_path="data/parking-garage.g2o")
    g2o_3d.read_g2o_3d()
    print(g2o_3d.poses[0])
    print(g2o_3d.edges[0,1])

