from Csv_Manip import read_csv_data
import numpy as np
from pyquaternion import Quaternion
import time
import matplotlib.pyplot as plt
from Quat_Funcs import *
from pytransform3d.rotations import *
from pykalman import KalmanFilter


def update_velocity(v, a, dt):
    return v + a * dt


def update_position(v, a, dt, pos):
    return 0.5 * a * dt ** 2 + v * dt + pos


def normalizeQuats(quatTable):
    i = 0
    normalizedQuatTable = np.zeros_like(quatTable)

    for row in quatTable:
        quaternionRow = Quaternion(row[1::])
        quaternionRowNormalized = quaternionRow.normalised

        quaternionRowNormalizedElements = quaternionRowNormalized.elements

        rowToAdd = np.insert(quaternionRowNormalizedElements, 0, row[0], axis=0)

        normalizedQuatTable[i, :] = rowToAdd

        i += 1

    return normalizedQuatTable


def invertQuats(quatTable):
    i = 0
    invertedQuats = np.zeros_like(quatTable)

    for row in quatTable:
        quaternionRow = Quaternion(row[1::])
        quaternionRowInverted = quaternionRow.inverse

        quaternionRowInvertedElements = quaternionRowInverted.elements

        rowToAdd = np.insert(quaternionRowInvertedElements, 0, row[0], axis=0)
        # print(type(rowToAdd))

        invertedQuats[i, :] = rowToAdd

        # time_vs_inv_quat[i, :] = [row[0], quaternionRowNormalizedElements]
        i += 1

    return invertedQuats


def plotQuats(quatTable, clubLen):
    
    fig = plt.figure()
    ax = plt.axes(projection="3d")
    x = np.array([clubLen, 0, 0])
    numRows = np.shape(quatTable)[0]
    posArr = np.zeros((numRows, 3))

    for t in range(len(quatTable)):

        R = matrix_from_quaternion(quatTable[t, 1::])
        posArr[t] = np.matmul(R, x)

    xs = posArr[:, 0]
    ys = posArr[:, 1]
    zs = posArr[:, 2]

    return xs, ys, zs

    # ax.scatter(xs, ys, zs, color="green")
    # ax.plot3D(xs, ys, zs)

    # plt.show()
    # ax.show()


# plot_basis()

# ax = None
# for t in range(len(quatTable)):
#     if t % 4 == 0:
#         R = matrix_from_quaternion(quatTable[t, 1::])
#         x = np.array([1, 0, 0])
#         p = 2 * (t / (len(quatTable) - 1) - 0.5) * x
#         ax = plot_basis(ax=ax, s=0.15, R=R, p=p)
# plt.tight_layout()
# plt.show()


def integrateAccel(dts, accel):
    prevVelocity = 0
    velocity = np.zeros_like(accel)
    for i in range(len(accel) - 1):
        velocity[i] = update_velocity(prevVelocity, accel[i], dts[i])
        prevVelocity = velocity[i]
    return velocity


def integrateVelocity(dts, accel, velocity):
    prevPos = 0
    pos = np.zeros_like(velocity)
    for i in range(len(velocity) - 1):
        pos[i] = update_position(velocity[i], accel[i], dts[i], prevPos)
        prevPos = pos[i]
    return pos


def calculatePosition(accelTable):

    time, xAccel, yAccel, zAccel = findAccel(accelTable)

    dts = np.zeros(len(time) - 1)
    for i in range(len(time) - 1):
        dts[i] = time[i + 1] - time[i]

    xVel = integrateAccel(dts, xAccel)
    yVel = integrateAccel(dts, yAccel)
    zVel = integrateAccel(dts, zAccel)

    plt.figure(0)

    # plot lines
    plt.plot(time, xVel, label="x velocity")
    plt.plot(time, yVel, label="y velocity")
    plt.plot(time, zVel, label="z velocity")
    plt.show()

    xPos = integrateVelocity(dts, xAccel, xVel)
    yPos = integrateVelocity(dts, yAccel, yVel)
    zPos = integrateVelocity(dts, zAccel, zVel)

    plt.figure(1)
    plt.plot(time, xPos, label="x velocity")
    plt.plot(time, yPos, label="y velocity")
    plt.plot(time, zPos, label="z velocity")
    plt.show()


def findAccel(accelTable):
    # create data
    x = accelTable[:, 0]

    y1 = applyKalmanFilter(accelTable[:, 1])
    y2 = applyKalmanFilter(accelTable[:, 2])
    y3 = applyKalmanFilter(accelTable[:, 3])

    
    # # plot lines
    # plt.plot(x, y1, label="x accel")
    # plt.plot(x, y2, label="y accel")
    # plt.plot(x, y3, label="z accel")

    # plt.legend()
    # plt.show()
    return x, y1, y2, y3


def applyKalmanFilter(accelData):
    # Time = [i * 0.015 for i in range(len(accelData))]
    # time step
    dt = 0.015

    # transition_matrix
    F = [[1, dt, 0.5 * dt ** 2], [0, 1, dt], [0, 0, 1]]

    # observation_matrix
    H = [0, 0, 1]

    # transition_covariance
    Q = [[0.5, 0, 0], [0, 0.5, 0], [0, 0, 0.5]]

    # observation_covariance
    R = np.var(accelData)

    # initial_state_mean
    X0 = [0, 0, 0]

    # initial_state_covariance
    P0 = [[0, 0, 0], [0, 0, 0], [0, 0, R]]

    n_timesteps = len(accelData)
    n_dim_state = 3
    filtered_state_means = np.zeros((n_timesteps, n_dim_state))
    filtered_state_covariances = np.zeros((n_timesteps, n_dim_state, n_dim_state))

    kf = KalmanFilter(
        transition_matrices=F,
        observation_matrices=H,
        transition_covariance=Q,
        observation_covariance=R,
        initial_state_mean=X0,
        initial_state_covariance=P0,
    )

    # iterative estimation for each new measurement
    for t in range(n_timesteps):
        if t == 0:
            filtered_state_means[t] = X0
            filtered_state_covariances[t] = P0
        else:
            filtered_state_means[t], filtered_state_covariances[t] = kf.filter_update(
                filtered_state_means[t - 1],
                filtered_state_covariances[t - 1],
                accelData[t],
            )

    return filtered_state_means[:, 2]

    # f, axarr = plt.subplots(3, sharex=True)

    # axarr[0].plot(Time, accelData, label="Input AccX")
    # axarr[0].plot(Time, filtered_state_means[:, 2], "r-", label="Estimated AccX")
    # axarr[0].set_title("Acceleration X")
    # axarr[0].grid()
    # axarr[0].legend()

    # # axarr[1].plot(Time, RefVelX, label="Reference VelX")
    # axarr[1].plot(Time, filtered_state_means[:, 1], "r-", label="Estimated VelX")
    # axarr[1].set_title("Velocity X")
    # axarr[1].grid()
    # axarr[1].legend()

    # # axarr[2].plot(Time, RefPosX, label="Reference PosX")
    # axarr[2].plot(Time, filtered_state_means[:, 0], "r-", label="Estimated PosX")
    # axarr[2].set_title("Position X")
    # axarr[2].grid()
    # axarr[2].legend()

    # plt.show()

def findMaxSpeed(dts, xs, ys, zs):
    maxSpeed = 0
    for i in range(len(dts)-1):

        dist = np.sqrt((xs[i+1]-xs[i])**2 + (ys[i+1]-ys[i])**2 + (zs[i+1]-zs[i])**2)
        tempSpeed = dist/(dts[i]/1000)

        if tempSpeed > maxSpeed:
            maxSpeed = tempSpeed

    return maxSpeed
        
        


    

def findMaxAccel(t, xAcc, yAcc, zAcc):
    maxAccel = 0
    for i in range(len(t)):
        tempAccel = np.sqrt(xAcc[i]**2 + yAcc[i]**2 + zAcc[i]**2)
        if tempAccel > maxAccel:
            maxAccel = tempAccel

    return maxAccel


def processRawData(data):
    # t0 = time.clock()
    data[:, 11] = data[:, 11] - data[0,11]
    print(np.shape(data))

    timeVsQuat = data[:, [11, 4, 5, 6, 7]]
    timeVsLinAccel = data[:, [11, 8, 9, 10]]
    timeVsEulerAngs = data[:, [11, 12, 13, 14]]
    timeVsInvQuat = np.zeros_like(timeVsQuat)
    timeVsQuat = normalizeQuats(timeVsQuat)
    timeVsInvQuat = invertQuats(timeVsQuat)

    return timeVsQuat, timeVsLinAccel, timeVsEulerAngs, timeVsInvQuat

    # plotQuats(timeVsQuat)
    # calculatePosition(timeVsLinAccel)
    # applyKalmanFilter(timeVsLinAccel[:, 3])


# np.set_printoptions(suppress=True)
# raw_data = read_csv_data("all_data_trimmed.csv")

# 0-3:   Calibration accel, gyro, magnetometer, system
# 4-7:   Quaternion w, x, y, z
# 8-10:  linear acceleration
# 11:    time since program start in ms
# 12-14: pitch, roll, yaw

# time is in milliseconds

# processRawData(raw_data)
