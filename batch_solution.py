# pylint: disable=invalid-name, E1101

from __future__ import print_function

import gtsam
import gtsam.utils.plot as gtsam_plot
import matplotlib.pyplot as plt
import numpy as np
# from gtsam.utils import plot

import read_g2o

def batch_solution_2d(g2o_2d):
    # read g2o data as dictionary

    # Create an empty nonlinear factor graph
    graph = gtsam.NonlinearFactorGraph()

    # prior mean and prior covariance, prior at origin
    priorMean = gtsam.Pose2(g2o_2d.poses[0]['CURRENT_ESTIMATE'][0],
                            g2o_2d.poses[0]['CURRENT_ESTIMATE'][1],
                            g2o_2d.poses[0]['CURRENT_ESTIMATE'][2])
    prior_noise = gtsam.noiseModel.Gaussian.Information(g2o_2d.edges[(0,1)]["INFO_MATRIX"])
    # prior_noise = gtsam.noiseModel.Gaussian.Covariance( np.linalg.inv(g2o_2d.edges[(0,1)]["INFO_MATRIX"]) )
    graph.add(gtsam.PriorFactorPose2(0, priorMean, prior_noise))

    # odometry
    for edge in g2o_2d.edges.values():
        key0 = edge['ID_SET'][0]
        key1 = edge['ID_SET'][1]
        odometry = gtsam.Pose2(edge['MEASUREMENT'][0], edge['MEASUREMENT'][1], edge['MEASUREMENT'][2])
        odometry_noise = gtsam.noiseModel.Gaussian.Information(edge['INFO_MATRIX'])
        # odometry_noise = gtsam.noiseModel.Gaussian.Covariance( np.linalg.inv(edge['INFO_MATRIX']) )

        graph.add(gtsam.BetweenFactorPose2(key0, key1, odometry, odometry_noise))

    # print("\nFactor Graph:\n{}".format(graph))

    # iteratively insert poses
    initial = gtsam.Values()
    for pose in g2o_2d.poses.values():
        # print(pose)
        key = pose['ID']
        estimate = gtsam.Pose2(pose['CURRENT_ESTIMATE'][0], pose['CURRENT_ESTIMATE'][1], pose['CURRENT_ESTIMATE'][2])
        initial.insert(key, estimate)

    # Gauss-Newton method optimization
    params = gtsam.GaussNewtonParams()
    optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
    result = optimizer.optimize()
    
    # dim = result.dim()/3
    # lenth = len(g2o_2d.poses)
    # print(dim - lenth)

    # marginals = gtsam.Marginals(graph, result)
    x_un = np.zeros(len(g2o_2d.poses))
    y_un = np.zeros(len(g2o_2d.poses))
    for i in range(len(g2o_2d.poses)):
        x_un[i] = initial.atPose2(i).x()
        y_un[i] = initial.atPose2(i).y()

    x_op = np.zeros(len(g2o_2d.poses))
    y_op = np.zeros(len(g2o_2d.poses))
    for i in range(len(g2o_2d.poses)):
        # gtsam_plot.plot_pose2(0, result.atPose2(i), 0.5,
        #                       marginals.marginalCovariance(i))
        # gtsam_plot.plot_pose2(1, result.atPose2(i), 0.01,
        #                       None)
        x_op[i] = result.atPose2(i).x()
        y_op[i] = result.atPose2(i).y()
        
    plt.plot(x_un,y_un,x_op,y_op,)
    plt.legend(["original","optimized"])
    plt.axis('equal')
    plt.show()



def batch_solution_3d(g2o_3d):
    graph = gtsam.NonlinearFactorGraph()
    prior_R = gtsam.Rot3(g2o_3d.poses[0]['QUATERNION'][3],
                         g2o_3d.poses[0]['QUATERNION'][0],
                         g2o_3d.poses[0]['QUATERNION'][1],
                         g2o_3d.poses[0]['QUATERNION'][2])
    prior_t = g2o_3d.poses[0]['CURRENT_ESTIMATE']
    priorMean = gtsam.Pose3( r=prior_R, t=np.array( prior_t,dtype=np.float64).reshape(3,1) )
    prior_noise = gtsam.noiseModel.Gaussian.Information(g2o_3d.edges[(0,1)]["INFO_MATRIX"])
    graph.add(gtsam.PriorFactorPose3(0, priorMean, prior_noise))

    # odometry
    for edge in g2o_3d.edges.values():
        id1, id2 = edge['ID_SET']
        Rotation = gtsam.Rot3(edge['MEASREMENT_QUATERNION'][3],
                              edge['MEASREMENT_QUATERNION'][0],
                              edge['MEASREMENT_QUATERNION'][1],
                              edge['MEASREMENT_QUATERNION'][2])
        trans = edge['MEASUREMENT']
        IMU_odometry = gtsam.Pose3( r=Rotation, t=np.array(trans,dtype=np.float64).reshape(3,1) )
        IMU_noise = gtsam.noiseModel.Gaussian.Information(edge['INFO_MATRIX'])
        graph.add(gtsam.BetweenFactorPose3(id1, id2, IMU_odometry, IMU_noise))
    
    initial = gtsam.Values()
    
    for pose in g2o_3d.poses.values():
        id = pose['ID']
        Rotation = gtsam.Rot3(pose['QUATERNION'][3],
                              pose['QUATERNION'][0],
                              pose['QUATERNION'][1],
                              pose['QUATERNION'][2])
        trans = pose['CURRENT_ESTIMATE']
        estimate = gtsam.Pose3( r=Rotation, t=np.array(trans, dtype=np.float64).reshape(3,1) )
        initial.insert(id, estimate)
    
    params = gtsam.GaussNewtonParams()
    optimizer = gtsam.GaussNewtonOptimizer(graph, initial, params)
    result = optimizer.optimize()


    resultPoses = gtsam.utilities.allPose3s(result)
    initialPose = gtsam.utilities.allPose3s(initial)
    x_un = np.array( [initialPose.atPose3(i).x() for i in range(initialPose.size())] )
    y_un = np.array( [initialPose.atPose3(i).y() for i in range(initialPose.size())] )
    z_un = np.array( [initialPose.atPose3(i).z() for i in range(initialPose.size())] )

    x_op = np.array( [resultPoses.atPose3(i).x() for i in range(resultPoses.size())] )
    y_op = np.array( [resultPoses.atPose3(i).y() for i in range(resultPoses.size())] )
    z_op = np.array( [resultPoses.atPose3(i).z() for i in range(resultPoses.size())] )


    # resultPoses = gtsam.utilities.allPose3s(result)
    # for i in range(resultPoses.size()):
    #     gtsam_plot.plot_pose3(1, resultPoses.atPose3(i),0.05)
    # plt.show()

    ax1 = plt.figure(1).add_subplot(projection='3d')
    # ax2 = plt.figure(2).add_subplot(projection='3d')

    ax1.plot(x_un, y_un, z_un, label="original")
    ax1.plot(x_op, y_op, z_op, label="optimized")
    ax1.legend()
    plt.show()


if __name__ == "__main__":
    # g2o_2d = read_g2o.G2O_data(g2o_path="data/input_INTEL_g2o.g2o")
    # g2o_2d.read_g2o_2d()
    # batch_solution_2d(g2o_2d)

    g2o_3d = read_g2o.G2O_data(g2o_path="data/parking-garage.g2o")
    g2o_3d.read_g2o_3d()
    batch_solution_3d(g2o_3d)