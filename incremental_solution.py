from __future__ import print_function

import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np

import read_g2o

def incremental_solution_2d(g2o_2d):
    # read g2o data as dictionary


    # g2o_2d.edges
    # g2o_2d.poses

    isam = gtsam.ISAM2()

    # prior noise
    prior_noise = gtsam.noiseModel.Gaussian.Information(g2o_2d.edges[(0,1)]["INFO_MATRIX"])

    for pose in g2o_2d.poses.values():
        graph = gtsam.NonlinearFactorGraph()
        initial = gtsam.Values()
        id, x, y, theta = pose['ID'], pose['CURRENT_ESTIMATE'][0], pose['CURRENT_ESTIMATE'][1], pose['CURRENT_ESTIMATE'][2]
        if id == 0:
            priorNoise = prior_noise
            estimate = gtsam.Pose2(x, y, theta)
            graph.add(gtsam.PriorFactorPose2(0, estimate, priorNoise))
            initial.insert(id, estimate)
        else:
            prevPose = result.atPose2(id-1)
            initial.insert(id, prevPose)
            for edge in g2o_2d.edges.values():
                id1, id2, dx, dy, dtheta, info = edge['ID_SET'][0], edge['ID_SET'][1], edge['MEASUREMENT'][0], edge['MEASUREMENT'][1], edge['MEASUREMENT'][2], edge['INFO_MATRIX']
                measurement = gtsam.Pose2(dx, dy, dtheta)
                if id == id2:
                    odometryNoise = gtsam.noiseModel.Gaussian.Information(info)
                    graph.add(gtsam.BetweenFactorPose2(id1, id2, measurement, odometryNoise))
        isam.update(graph, initial)
        result = isam.calculateEstimate()

    # plot and compare
    x_un = np.zeros(len(g2o_2d.poses))
    y_un = np.zeros(len(g2o_2d.poses))
    for i in range(len(g2o_2d.poses)):
        x_un[i] = g2o_2d.poses[i]['CURRENT_ESTIMATE'][0]
        y_un[i] = g2o_2d.poses[i]['CURRENT_ESTIMATE'][1]

    x_op = np.zeros(len(g2o_2d.poses))
    y_op = np.zeros(len(g2o_2d.poses))
    for i in range(len(g2o_2d.poses)):
        x_op[i] = result.atPose2(i).x()
        y_op[i] = result.atPose2(i).y()

    plt.plot(x_un,y_un,x_op,y_op,)
    plt.legend(["original","optimized"])
    plt.axis('equal')
    plt.show()


def incremental_solution_3d(g2o_3d):
    isam = gtsam.ISAM2()

    # prior noise
    prior_noise = gtsam.noiseModel.Gaussian.Information(g2o_3d.edges[(0,1)]["INFO_MATRIX"])

    for pose in g2o_3d.poses.values():
        graph = gtsam.NonlinearFactorGraph()
        initial = gtsam.Values()

        id = pose['ID']
        Rotation = gtsam.Rot3(pose['QUATERNION'][3],
                              pose['QUATERNION'][0],
                              pose['QUATERNION'][1],
                              pose['QUATERNION'][2])
        trans = pose['CURRENT_ESTIMATE']

        if id == 0:
            priorNoise = prior_noise
            estimate = gtsam.Pose3( r=Rotation, t=np.array(trans, dtype=np.float64).reshape(3,1) )
            graph.add(gtsam.PriorFactorPose3(0, estimate, priorNoise))
            initial.insert(id, estimate)
        else:
            prevPose = result.atPose3(id-1)
            initial.insert(id, prevPose)
            for edge in g2o_3d.edges.values():
                # id1, id2, dx, dy, dtheta, info = edge['ID_SET'][0], edge['ID_SET'][1], edge['MEASUREMENT'][0], edge['MEASUREMENT'][1], edge['MEASUREMENT'][2], edge['INFO_MATRIX']
                id1, id2 = edge['ID_SET']
                Rotation = gtsam.Rot3(edge['MEASREMENT_QUATERNION'][3],
                                      edge['MEASREMENT_QUATERNION'][0],
                                      edge['MEASREMENT_QUATERNION'][1],
                                      edge['MEASREMENT_QUATERNION'][2])
                trans = edge['MEASUREMENT']
                
                IMU_odometry = gtsam.Pose3( r=Rotation, t=np.array(trans,dtype=np.float64).reshape(3,1) )
                if id == id2:
                    IMU_noise = gtsam.noiseModel.Gaussian.Information(edge['INFO_MATRIX'])
                    graph.add(gtsam.BetweenFactorPose3(id1, id2, IMU_odometry, IMU_noise))
        isam.update(graph, initial)
        result = isam.calculateEstimate()

    # resultPoses = gtsam.utilities.allPose3s(result)
    # for i in range(resultPoses.size()):
    #     gtsam_plot.plot_pose3(1, resultPoses.atPose3(i),0.05)
    # plt.show()
     
    resultPoses = gtsam.utilities.allPose3s(result)
    # initialPose = gtsam.utilities.allPose3s(initial)
    # x_un = np.array( [initialPose.atPose3(i).x() for i in range(initialPose.size())] )
    # y_un = np.array( [initialPose.atPose3(i).y() for i in range(initialPose.size())] )
    # z_un = np.array( [initialPose.atPose3(i).z() for i in range(initialPose.size())] )

    x_un = np.zeros(len(g2o_3d.poses))
    y_un = np.zeros(len(g2o_3d.poses))
    z_un = np.zeros(len(g2o_3d.poses))
    for i in range(len(g2o_3d.poses)):
        x_un[i] = g2o_3d.poses[i]['CURRENT_ESTIMATE'][0]
        y_un[i] = g2o_3d.poses[i]['CURRENT_ESTIMATE'][1]
        z_un[i] = g2o_3d.poses[i]['CURRENT_ESTIMATE'][2]

    x_op = np.array( [resultPoses.atPose3(i).x() for i in range(resultPoses.size())] )
    y_op = np.array( [resultPoses.atPose3(i).y() for i in range(resultPoses.size())] )
    z_op = np.array( [resultPoses.atPose3(i).z() for i in range(resultPoses.size())] )

    ax1 = plt.figure(1).add_subplot(projection='3d')

    ax1.plot(x_un, y_un, z_un, label="original")
    ax1.plot(x_op, y_op, z_op, label="optimized")
    ax1.legend()
    plt.show()    


if __name__ == '__main__':
    # g2o_2d = read_g2o.G2O_data(g2o_path="data/input_INTEL_g2o.g2o")
    # g2o_2d.read_g2o_2d()
    # incremental_solution_2d(g2o_2d)

    g2o_3d = read_g2o.G2O_data(g2o_path="data/parking-garage.g2o")
    g2o_3d.read_g2o_3d()
    incremental_solution_3d(g2o_3d)
            


