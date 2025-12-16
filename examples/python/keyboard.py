import numpy as np
from time import perf_counter_ns, sleep

import trakr_sdk as sdk
from trakr_sdk import QuadDataTypes, AlliedDataTypes
from trakr_sdk import motion_configs as MotionDataTypes

import threading

ROBOT_IP = "192.168.3.50"
ROBOT_PORT = 15251
FREQ = 400
LOOP_TIME_NS = 1e9/FREQ

plan = AlliedDataTypes.Plan()
state = AlliedDataTypes.State()
config = QuadDataTypes.CONFIG_SET()

set_config : bool = False

running_ = True
changed = False

var = 0

class Speeds:
    FORWARD_VEL = 0.3
    FORWARD_RUN = 0.6
    FORWARD_DASH = 0.8
    FORWARD_NEXT = 1.0
    BACKWARD_VEL = -0.3
    TROT_VEL = 0.02
    RIGHT_VEL = 0.3
    LEFT_VEL = -0.3
    SMALL_TURN_ACK = 0.05
    SMALL_TURN_CK = -0.05
    BIG_TURN_ACK = 0.5
    BIG_TURN_CK = -0.5
    TRIAL_VEL = 0.001
    LOOK_UP = 0.1
    LOOK_UP_BIG = 0.15
    LOOK_NET = 0.1


def user_input():
    global running_, changed, var

    while(running_):
        varbuf = input("Enter Command")
        try:
            var = int(varbuf)
        except:
            var = 0

        changed = True
        
def doAction():
    global var, set_config, changed

    # Basic Speeds
    if(var == 0):
        plan.torso.vel = np.zeros(6)
    elif(var == 1):
        plan.torso.vel = np.zeros(6)
        plan.torso.vel[4] = Speeds.FORWARD_VEL
    elif(var == 2):
        plan.torso.vel = np.zeros(6)
        plan.torso.vel[4] = Speeds.BACKWARD_VEL
    elif(var == 3):
        plan.torso.vel = np.zeros(6)
        plan.torso.vel[4] = Speeds.TROT_VEL
    elif(var == 4):
        plan.torso.vel = np.zeros(6)
        plan.torso.vel[3] = Speeds.RIGHT_VEL
    elif(var == 5):
        plan.torso.vel = np.zeros(6)
        plan.torso.vel[3] = Speeds.LEFT_VEL
    elif(var == 6):
        plan.torso.vel[2] = Speeds.SMALL_TURN_ACK
    elif(var == 7):
        plan.torso.vel[2] = Speeds.SMALL_TURN_CK
    elif(var == 8):
        plan.torso.vel = np.zeros(6)
        plan.torso.vel[2] = Speeds.BIG_TURN_ACK
    elif(var == 9):
        plan.torso.vel = np.zeros(6)
        plan.torso.vel[2] = Speeds.BIG_TURN_CK

    # Higher speeds
    elif(var == 93):
        plan.torso.vel = np.zeros(6)
        plan.torso.vel[4] = Speeds.FORWARD_RUN
    elif(var == 94):
        plan.torso.vel = np.zeros(6)
        plan.torso.vel[4] = Speeds.FORWARD_DASH
    elif(var == 95):
        plan.torso.vel = np.zeros(6)
        plan.torso.vel[4] = Speeds.FORWARD_NEXT

    # Mode Switching
    elif(var == 31):
        config.motion.planner = MotionDataTypes.TaskTypes.eGesture
        config.motion.sequence.type = MotionDataTypes.GestureTypes.eOrientation
        if(changed):
            set_config = True
            changed = False
    elif(var == 32):
        config.motion.planner = MotionDataTypes.TaskTypes.eMotion
        config.motion.strategy.type = MotionDataTypes.MotionModes.eClassicalMode
        config.motion.strategy.seq = MotionDataTypes.MotionGaits.eStance
        if(changed):
            set_config = True
            changed = False
    elif(var == 33):
        config.motion.planner = MotionDataTypes.TaskTypes.eMotion
        config.motion.strategy.type = MotionDataTypes.MotionModes.eAIMode
        if(changed):
            set_config = True
            changed = False
    elif(var == 34):
        config.motion.planner = MotionDataTypes.TaskTypes.eMotion
        config.motion.strategy.type = MotionDataTypes.MotionModes.eDeveloperMode
        if(changed):
            set_config = True
            changed = False
    elif(var == 39):
        config.master.doExit()
        if(changed):
            set_config = True
            changed = False
    elif(var == 41):
        config.motion.planner = MotionDataTypes.TaskTypes.eGesture
        config.motion.sequence.type = MotionDataTypes.GestureTypes.eLeftHandShake
        if(changed):
            set_config = True
            changed = False
    elif(var == 42):
        config.motion.planner = MotionDataTypes.TaskTypes.eGesture
        config.motion.sequence.type = MotionDataTypes.GestureTypes.eRightHandShake
        if(changed):
            set_config = True
            changed = False
    elif(var == 90):
        config.master.doShutDown()
        if(changed):
            set_config = True
            changed = False
    elif(var == 101):
        config.master.kill()
        if(changed):
            set_config = True
            changed = False

    else:
        plan.torso.vel = np.zeros(6)
    
    if(config.motion.planner == MotionDataTypes.TaskTypes.eMotion):
        if(config.motion.strategy.type == MotionDataTypes.MotionModes.eClassicalMode):
            if(np.linalg.norm(plan.torso.vel) < 0.02):
                config.motion.strategy.seq = MotionDataTypes.MotionGaits.eStance
                set_config = True
            else:
                config.motion.strategy.seq = MotionDataTypes.MotionGaits.eTrot
                set_config = True

def main():
    global set_config, running_

    config.safety.llheartbeat.disable()

    user_ = threading.Thread(target=user_input)

    robot = sdk.Robot(ROBOT_IP, ROBOT_PORT)

    if(not robot.setup(config, plan)):
        print("[MAIN] Failed to setup robot")
        return False

    config_status : int = 0

    set_config = False

    user_.start()

    while(robot.isAlive()):
        time_s = perf_counter_ns()
        if(not robot.run()):
            break

        robot.getData(state)
        robot.getConfig(config)

        # this is feedback for last config set.
        # 0 . no config change was requested
        # 1 . Accepted
        # 2 . Rejected 
        config_status = robot.getConfigStatus()

        doAction()

        if(set_config):
            robot.setConfig(config)
            set_config = False

        robot.setData(plan)

        delta_t = perf_counter_ns() - time_s
        if(delta_t < LOOP_TIME_NS):
            sleep((LOOP_TIME_NS - delta_t)/1e9)

    running_ = False
    user_.join()

    return 0

if __name__ == "__main__":
    main()

