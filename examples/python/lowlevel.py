import numpy as np
from time import perf_counter_ns, sleep

import trakr_sdk as sdk
from trakr_sdk import QuadDataTypes, AlliedDataTypes, NDOF
from trakr_sdk import motion_configs as MotionDataTypes

import torch

ROBOT_IP = "192.168.3.50"
ROBOT_PORT = 15251
FREQ = 400

start_time = 0

plan = AlliedDataTypes.Plan()
state = AlliedDataTypes.State()
config = QuadDataTypes.CONFIG_SET()

STANDUP_TIME = 2.0
SITDOWN_TIME = 2.0
RUN_TIME = 40.0

standing_angles = np.array([
    0, 0.81, -1.57,
    0, 0.81, -1.57,
    0, 0.81, -1.57,
    0, 0.81, -1.57,
])

initial_angles = None
postrun_angles = None

class Rate:
    def __init__(self, freq : int):
        self.freq = freq
        self.loop_time_ns = int(1e9/freq)

        self.time = perf_counter_ns()

    def sleep(self):
        time_t = perf_counter_ns()
        delta_t = time_t - self.time
        self.time = time_t
        if(delta_t < self.loop_time_ns):
            sleep((self.loop_time_ns - delta_t)/1e9)


class Actor:
    FREQ = 50

    LIN_VEL_SCALE = 2.0
    ANG_VEL_SCALE = 0.25
    DOF_POS_SCALE = 1.0
    DOF_VEL_SCALE = 0.05

    ACTION_SCALE = 0.5

    def __init__(self):
        # load models here
        self.model = None

        self.time = perf_counter_ns()
        self.dt = 1./self.FREQ

        self.kp = 20.
        self.kd = 2.

        self.n_obs = 48
        self.n_actions = 12

        self.observations = np.zeros(self.n_obs)
        self.actions = np.zeros(self.n_actions)

        self.gravity = np.array([0, 0, -1])
        self.command = np.array([0, 0, 0])

        self.model = torch.jit.load("policy_ppo_1.pt")

    def _rotationMatrix(self, euler : np.ndarray) -> np.ndarray:
        """
        Calculates Rotation Matrix from extrinsic XYZ Euler angles (radians).
        theta1, theta2, theta3 are angles for X, Y, and Z axes respectively.
        """
        c1, s1 = np.cos(euler[0]), np.sin(euler[0])
        c2, s2 = np.cos(euler[1]), np.sin(euler[1])
        c3, s3 = np.cos(euler[2]), np.sin(euler[2])

        R_x = np.array([[1, 0, 0],
                        [0, c1, -s1],
                        [0, s1, c1]])

        R_y = np.array([[c2, 0, s2],
                        [0, 1, 0],
                        [-s2, 0, c2]])

        R_z = np.array([[c3, -s3, 0],
                        [s3, c3, 0],
                        [0, 0, 1]])

        # Combined rotation matrix is R_z * R_y * R_x
        R = R_z @ R_y @ R_x
        return R

    def _getTime(self):
        return (perf_counter_ns() - self.time)/1e9
    
    def _resetTime(self):
        self.time = perf_counter_ns()

    def run(self, state: AlliedDataTypes.State, plan: AlliedDataTypes.Plan):
        # doing inference at fixed frequency/dt
        if(self._getTime() > self.dt):
            # Creating observations for policy
            self.setupInput(state)

            # doing forward inference of model
            self.forward()

            # parsing actions of policy
            self.setupOutput(plan)

            # Resetting time for matching freq
            self._resetTime()

    def JointInputAdapter(self, data : np.ndarray):
        data2 = np.zeros(NDOF)
        data2[0] = -data[6]
        data2[1] = -data[7]
        data2[2] = -data[8]
        data2[3] = -data[0]
        data2[4] = -data[1]
        data2[5] = -data[2]
        data2[6] = -data[9]
        data2[7] = -data[10]
        data2[8] = -data[11]
        data2[9] = -data[3]
        data2[10] = -data[4]
        data2[11] = -data[5]
        return data2
    
    def JointOutputAdapter(self, data : np.ndarray):
        data2 = np.zeros(NDOF)
        data2[0] = -data[3]
        data2[1] = -data[4]
        data2[2] = -data[5]
        data2[3] = -data[9]
        data2[4] = -data[10]
        data2[5] = -data[11]
        data2[6] = -data[0]
        data2[7] = -data[1]
        data2[8] = -data[2]
        data2[9] = -data[6]
        data2[10] = -data[7]
        data2[11] = -data[8]
        return data2

    def setupInput(self, state: AlliedDataTypes.State):
        rot_mat : np.ndarray = self._rotationMatrix(state.imu.euler)

        self.observations[0:3] = rot_mat.transpose() @ state.torso.vel[3:6] * self.LIN_VEL_SCALE # lin vel
        self.observations[3:6] = state.imu.gyro * self.ANG_VEL_SCALE # ang vel
        self.observations[6:9] = rot_mat.transpose() @ np.array([0, 0, -1]) # proj gravity
        self.observations[9:12] = self.command # commands
        self.observations[12:24] = self.JointInputAdapter(state.joint.pos[0:12] - standing_angles) * self.DOF_POS_SCALE
        self.observations[24:36] = self.JointInputAdapter(state.joint.vel[0:12]) * self.DOF_VEL_SCALE
        self.observations[36:48] = self.actions[0:12]

        self.observations[5] *= 0.5

    def setupOutput(self, plan: AlliedDataTypes.Plan):
        plan.joint.pos[0:12] = (self.JointOutputAdapter(self.actions[0:12]) * self.ACTION_SCALE) + standing_angles
        plan.joint.vel = np.zeros(NDOF)
        plan.joint.kp = np.ones(NDOF)*self.kp
        plan.joint.kd = np.ones(NDOF)*self.kd
        plan.joint.tor = np.zeros(NDOF)

    def forward(self):
        obs = torch.from_numpy(self.observations).view(1, -1).to(torch.float)
        self.actions = self.model(obs).detach().numpy()[0, :]

def getTimeFromStart():
    global start_time
    return (perf_counter_ns() - start_time)/1e9

        
def doAction(actor : Actor, state : AlliedDataTypes.State, plan : AlliedDataTypes.Plan):
    global initial_angles, postrun_angles

    if(getTimeFromStart() < STANDUP_TIME):
        if(type(initial_angles) != np.ndarray):
            initial_angles = np.array(state.joint.pos)
            print("Initial Angles:", initial_angles)
        # do bringup here
        plan.joint.pos = standUp(getTimeFromStart())
        plan.joint.vel = np.zeros(NDOF)
        plan.joint.kp = np.ones(NDOF)*60.
        plan.joint.kd = np.ones(NDOF)*2.
        plan.joint.tor = np.zeros(NDOF)

    elif(getTimeFromStart() > (STANDUP_TIME + RUN_TIME)):
        if(type(postrun_angles) != np.ndarray):
            postrun_angles = np.array(state.joint.pos)
        # do bringdown here
        plan.joint.pos = sitDown(getTimeFromStart() - (STANDUP_TIME + RUN_TIME))
        plan.joint.vel = np.zeros(NDOF)
        plan.joint.kp = np.ones(NDOF)*60.
        plan.joint.kd = np.ones(NDOF)*2.
        plan.joint.tor = np.zeros(NDOF)

    else:
        # do policy inference here
        actor.run(state, plan)

def standUp(time_t):
    return ((min(time_t, STANDUP_TIME) * (standing_angles - initial_angles)/(STANDUP_TIME)) + initial_angles)

def sitDown(time_t):
    return ((min(time_t, SITDOWN_TIME) * (initial_angles - postrun_angles)/(SITDOWN_TIME)) + postrun_angles)


def main():
    global start_time

    # to run loop at fixed freq
    rate = Rate(FREQ)

    # Creating our robot
    robot = sdk.Robot(ROBOT_IP, ROBOT_PORT, sdk.Mode.eLowLevel)

    if(not robot.setup(config, plan)):
        print("[MAIN] Failed to setup robot")
        return False
    
    # Setting up our policy/actor
    actor = Actor()

    # setting start time
    start_time = perf_counter_ns()

    while(robot.isAlive()):
        if(not robot.run()):
            print("Failed to run robot")
            break
        
        # Getting state from buffers
        robot.getData(state)

        # taking action
        doAction(actor, state, plan)

        # Setting plan to buffers
        robot.setData(plan)

        # Ensuring steady frequency of loop
        # rate.sleep()

    return 0

if __name__ == "__main__":
    main()

