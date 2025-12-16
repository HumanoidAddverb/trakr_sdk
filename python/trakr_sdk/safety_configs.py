from enum import Enum
import numpy as np

NDOF = 12

class Constants:
    SAFETY_JPOS_THRESHOLD_MIN = [
        -0.785,
        -1.57,
        -3.20,
        -0.785,
        -1.57,
        -3.20,
        -0.785,
        -1.57,
        -3.20,
        -0.785,
        -1.57,
        -3.20
    ]

    SAFETY_JPOS_THRESHOLD_MAX = [
        0.785,
        3.5,
        0.0,
        0.785,
        3.5,
        0.0,
        0.785,
        3.5,
        0.0,
        0.785,
        3.5,
        0.0 
    ]

    SAFETY_BASE_VEL_THRESH = 40.0

    SAFETY_BASE_TORQUE_THRESH = 50.0
    SAFETY_BASE_TORQUE_RECOMMENDED = 40.0

class BaseConfig:
    enabled : bool = True

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False

class JointSafetyConfig(BaseConfig):
    check_jpos : bool = True
    check_jvel : bool = True
    check_jtor : bool = True
    check_dtor : bool = True

    lim_jpos_min = np.zeros(NDOF)
    lim_jpos_max = np.zeros(NDOF)
    lim_jvel = np.zeros(NDOF)
    lim_jtor = np.zeros(NDOF)
    lim_dtor = np.zeros(NDOF)

    def __init__(self):
        for i in range(NDOF):
            self.lim_jpos_min[i] = Constants.SAFETY_JPOS_THRESHOLD_MIN[i]
            self.lim_jpos_max[i] = Constants.SAFETY_JPOS_THRESHOLD_MAX[i]

            self.lim_jvel[i] = Constants.SAFETY_BASE_VEL_THRESH
            self.lim_jtor[i] = Constants.SAFETY_BASE_TORQUE_RECOMMENDED
            self.lim_dtor[i] = Constants.SAFETY_BASE_TORQUE_RECOMMENDED

class JointHeartbeatConfig(BaseConfig):
    check_joints : bool = True
    joints_timeout = 100 # ms

    check_IMU : bool = True
    IMU_timeout = 200 # ms

class TorsoSafetyConfig(BaseConfig):
    check_roll = True
    roll_limit = 1.2 # rad

    check_pitch = True
    pitch_limit = 1.2 # rad

    check_gyro = False
    gyro_limit = 5.0 # rad/s

class CONFIG_SET:
    llsafety : JointSafetyConfig
    llheartbeat : JointHeartbeatConfig
    torso : TorsoSafetyConfig

    def __init__(self):
        self.llsafety = JointSafetyConfig()
        self.llheartbeat = JointHeartbeatConfig()
        self.torso = TorsoSafetyConfig()