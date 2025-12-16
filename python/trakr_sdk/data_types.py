import numpy as np

NDOF = 12

class JointState:
    pos = np.zeros(NDOF)
    vel = np.zeros(NDOF)
    tor = np.zeros(NDOF)
    kp = np.zeros(NDOF)
    kd = np.zeros(NDOF)

class TorsoState:
    pos = np.zeros(6)
    vel = np.zeros(6)

class IMUState:
    acc = np.zeros(3)
    gyro = np.zeros(3)
    mag = np.zeros(3)
    euler = np.zeros(3)

class Plan:
    torso = TorsoState()
    joint = JointState()

class State:
    torso = TorsoState()
    joint = JointState()
    imu = IMUState()

