import sys
from time import sleep
from enum import Enum

from .lib import trakr_python_sdk as sdk

from . import config_types as QuadDataTypes
from . import data_types as AlliedDataTypes

from .data_types import NDOF

class RobotMode(Enum):
    eLowLevel = 0
    eHighLevel = 1

class Robot:
    def __init__(self, ip : str, port : int, mode : int = RobotMode.eHighLevel):
        self.net_ = sdk.RobotNetwork()
        self.plan_ = sdk.AlliedPlan()
        self.state_ = sdk.AlliedState()

        self.setConfig_ : bool = False
        
        self.configStatus_ : int = 0

        self.ip = ip
        self.port = port
        self.mode = mode

    def setup(self, config : QuadDataTypes.CONFIG_SET, plan : AlliedDataTypes.Plan) -> bool:
        if(not self.net_.setup()):
            print("[ROBOT] Failed to setup Network")
            return False
        
        self._setConfigToPacket(config)
        self._setPlanToPacket(plan)
        self.plan_.config.status = 0

        if(not self.net_.setData(self.plan_)):
            print("[ROBOT] Failed to set data")
            return False
        
        if(not self.net_.connect(self.ip, self.port)):
            print("[ROBOT] Failed to connect to Network")
            return False
        
        print("[ROBOT] Connected!")

        self._bringUp()

        return True

    def run(self) -> bool:
        if(not self.net_.isConnected()):
            print("[ROBOT] Socket disconnected")
            return False

        if(self.net_.readData()):
            if(not self.net_.getData(self.state_)):
                print("[ROBOT] Unable to get data from Network")
                return False
            
            # Reset once acknowledged by server
            if(self.state_.config.status > 0):
                self.plan_.config.status = 0
                if(self.state_.config.status > 1):
                    self.plan_.config.status = -1
                self.configStatus_ = int(self.state_.config.status)
                self.state_.config.status = 0
            else:
                if(self.plan_.config.status < 0):
                    self.plan_.config.status = 0

        if(not self.net_.setData(self.plan_)):
            print("[ROBOT] Unable to set data to network")
            return False
        
        if(not self.net_.writeData()):
            print("[ROBOT] Failed to send data")
            return False
        
        return True
    
    def _bringUp(self) -> bool:
        print("[ROBOT] Waiting for Robot to BringUp!")

        self.state_.config.status = -1

        config_ = QuadDataTypes.CONFIG_SET()

        ctr = 0

        while((self.state_.config.status < 0) or (ctr < 10)):
            ctr += 1

            if(not self.run()):
                print("[ROBOT] Failed in BringUp")
                return False
            
            if(ctr < 10):
                self.getConfigStatus()
                sleep(0.01)
                continue

            # Some internal state machine to check status
            if(self.state_.config.status == -2):
                self.getConfig(config_)
                self.setConfig(config_)
                self.plan_.config.status = 1
            else:
                self.plan_.config.status = 0

            sleep(0.01)

        self.getConfigStatus()
        self.plan_.config.status = 0

        self._setMode(config_)

        print("[ROBOT] Robot BringUp Completed!")
        return True
    
    def _setMode(self, config : QuadDataTypes.CONFIG_SET):
        if(self.mode != RobotMode.eLowLevel):
            ctr = 0
            # Fetching latest data for some time (1s)
            while(self.isAlive() and (ctr < 50)):
                ctr += 1

                if(not self.run()):
                    print("[ROBOT] Failed in SetMode")
                    return False
                
                self.getConfigStatus()
                self.getConfig(config)

                sleep(0.02)
            return True
        
        ctr = 0
        # Fetching latest data for some time (2s)
        while(self.isAlive() and (ctr < 100)):
            ctr += 1

            if(not self.run()):
                print("[ROBOT] Failed in SetMode")
                return False
            
            self.getConfigStatus()
            self.getConfig(config)

            sleep(0.02)

        # Setting config for low-level mode
        while(self.isAlive()):
            if(not self.run()):
                print("[ROBOT] Failed in SetMode")
                return False

            if(self.getConfigStatus() == 2):
                break

            self.getConfig(config)
            config.motion.planner = QuadDataTypes.MotionDataTypes.TaskTypes.eMotion
            config.motion.strategy.type = QuadDataTypes.MotionDataTypes.MotionModes.eDeveloperMode
            self.setConfig(config)

            sleep(0.01)

        ctr = 0
        # Fetching latest data for some time (2s)
        while(self.isAlive() and (ctr < 125)):
            ctr += 1

            if(not self.run()):
                print("[ROBOT] Failed in SetMode")
                return False
            
            self.getConfigStatus()
            self.getConfig(config)

            sleep(0.02)
        print("After Loop")
        print("[ROBOT] Switched to LowLevel/Developer Mode")

        return True

    def isAlive(self) -> bool:
        return self.net_.isConnected()
    
    def setData(self, plan : AlliedDataTypes.Plan) -> bool:
        self._setPlanToPacket(plan)
        return True

    def getData(self, state : AlliedDataTypes.State) -> bool:
        self._getStateFromPacket(state)
        self.state_.config.status = 0
        return True

    def setConfig(self, config : QuadDataTypes.CONFIG_SET) -> bool:
        self._setConfigToPacket(config)
        if(self.plan_.config.status >= 0):
            self.plan_.config.status = 1
        return True

    def getConfig(self, config : QuadDataTypes.CONFIG_SET) -> bool:
        self._getConfigFromPacket(config)
        return True

    def getConfigStatus(self) -> int:
        ret = int(self.configStatus_)
        self.configStatus_ = 0
        return ret

    def _setPlanToPacket(self, plan : AlliedDataTypes.Plan):
        self.plan_.plan.joint.pos = plan.joint.pos
        self.plan_.plan.joint.vel = plan.joint.vel
        self.plan_.plan.joint.tor = plan.joint.tor
        self.plan_.plan.joint.kp = plan.joint.kp
        self.plan_.plan.joint.kd = plan.joint.kd

        self.plan_.plan.torso.pos = plan.torso.pos
        self.plan_.plan.torso.vel = plan.torso.vel


    def _setConfigToPacket(self, config : QuadDataTypes.CONFIG_SET):
        self.plan_.config.master.shutdown = config.master.shutdown
        self.plan_.config.master.exit = config.master.exit
        self.plan_.config.master.killed = config.master.killed

        self.plan_.config.motion.planner = config.motion.planner.value
        self.plan_.config.motion.sequence.type = config.motion.sequence.type.value
        self.plan_.config.motion.sequence.seq = 0
        self.plan_.config.motion.strategy.type = config.motion.strategy.type.value
        if(config.motion.strategy.type == QuadDataTypes.MotionDataTypes.MotionModes.eClassicalMode):
            self.plan_.config.motion.strategy.seq = config.motion.strategy.seq.value
        elif(config.motion.strategy.type == QuadDataTypes.MotionDataTypes.MotionModes.eAIMode):
            self.plan_.config.motion.strategy.seq = config.motion.strategy.seq.value
        else:
            self.plan_.config.motion.strategy.seq = 0

        self.plan_.config.safety.llsafety.enabled = config.safety.llsafety.enabled
        self.plan_.config.safety.llsafety.jpos = config.safety.llsafety.check_jpos
        self.plan_.config.safety.llsafety.jvel = config.safety.llsafety.check_jvel
        self.plan_.config.safety.llsafety.jtor = config.safety.llsafety.check_jtor
        self.plan_.config.safety.llsafety.dtor = config.safety.llsafety.check_dtor

        self.plan_.config.safety.llsafety.lim_jpos_min = config.safety.llsafety.lim_jpos_min
        self.plan_.config.safety.llsafety.lim_jpos_max = config.safety.llsafety.lim_jpos_max
        self.plan_.config.safety.llsafety.lim_jvel = config.safety.llsafety.lim_jvel
        self.plan_.config.safety.llsafety.lim_jtor = config.safety.llsafety.lim_jtor
        self.plan_.config.safety.llsafety.lim_dtor = config.safety.llsafety.lim_dtor

        self.plan_.config.safety.llheartbeat.enabled = config.safety.llheartbeat.enabled
        self.plan_.config.safety.llheartbeat.init_disable_time = 1
        self.plan_.config.safety.llheartbeat.enable_CAN = config.safety.llheartbeat.check_joints
        self.plan_.config.safety.llheartbeat.CAN_timeout = config.safety.llheartbeat.joints_timeout
        self.plan_.config.safety.llheartbeat.observe_jpos = False
        self.plan_.config.safety.llheartbeat.observe_jvel = True
        self.plan_.config.safety.llheartbeat.observe_jtor = True
        self.plan_.config.safety.llheartbeat.enable_IMU = config.safety.llheartbeat.check_IMU
        self.plan_.config.safety.llheartbeat.IMU_timeout = config.safety.llheartbeat.IMU_timeout
        self.plan_.config.safety.llheartbeat.observe_accel = True
        self.plan_.config.safety.llheartbeat.observe_gyro = True

        self.plan_.config.safety.torso.enabled = config.safety.torso.enabled
        self.plan_.config.safety.torso.observe_roll = config.safety.torso.check_roll
        self.plan_.config.safety.torso.roll_limit = config.safety.torso.roll_limit
        self.plan_.config.safety.torso.observe_pitch = config.safety.torso.check_pitch
        self.plan_.config.safety.torso.pitch_limit = config.safety.torso.pitch_limit
        self.plan_.config.safety.torso.observe_gyro = config.safety.torso.check_gyro
        self.plan_.config.safety.torso.gyro_limit = config.safety.torso.gyro_limit


    def _getStateFromPacket(self, state : AlliedDataTypes.State):
        for i in range(NDOF):
            state.joint.pos[i] = self.state_.state.joint.pos[i]
            state.joint.vel[i] = self.state_.state.joint.vel[i]
            state.joint.tor[i] = self.state_.state.joint.tor[i]
            state.joint.kp[i] = self.state_.state.joint.kp[i]
            state.joint.kd[i] = self.state_.state.joint.kd[i]
        state.joint.timestamp = self.state_.state.joint.timestamp

        for i in range(6):
            state.torso.pos[i] = self.state_.state.torso.pos[i]
            state.torso.vel[i] = self.state_.state.torso.vel[i]
        state.torso.timestamp = self.state_.state.torso.timestamp

        for i in range(3):
            state.imu.acc[i] = self.state_.state.imu.acc[i]
            state.imu.mag[i] = self.state_.state.imu.mag[i]
            state.imu.gyro[i] = self.state_.state.imu.gyro[i]
            state.imu.euler[i] = self.state_.state.imu.euler[i]
        state.imu.timestamp = self.state_.state.imu.timestamp

    def _getConfigFromPacket(self, config : QuadDataTypes.CONFIG_SET):
        config.master.shutdown = self.state_.config.master.shutdown
        config.master.exit = self.state_.config.master.exit
        config.master.killed = self.state_.config.master.killed

        config.motion.planner = QuadDataTypes.MotionDataTypes.TaskTypes(self.state_.config.motion.planner)
        config.motion.sequence.type = QuadDataTypes.MotionDataTypes.GestureTypes(self.state_.config.motion.sequence.type)
        config.motion.sequence.seq = 0
        config.motion.strategy.type = QuadDataTypes.MotionDataTypes.MotionModes(self.state_.config.motion.strategy.type)
        try:
            config.motion.strategy.seq = QuadDataTypes.MotionDataTypes.MotionGaits(self.state_.config.motion.strategy.seq)
        except: 
            config.motion.strategy.seq = QuadDataTypes.MotionDataTypes.MotionGaits(0)

        config.safety.llsafety.enabled = self.state_.config.safety.llsafety.enabled
        config.safety.llsafety.check_jpos = self.state_.config.safety.llsafety.jpos
        config.safety.llsafety.check_jvel = self.state_.config.safety.llsafety.jvel
        config.safety.llsafety.check_jtor = self.state_.config.safety.llsafety.jtor
        config.safety.llsafety.check_dtor = self.state_.config.safety.llsafety.dtor

        config.safety.llsafety.lim_jpos_min = self.state_.config.safety.llsafety.lim_jpos_min
        config.safety.llsafety.lim_jpos_max = self.state_.config.safety.llsafety.lim_jpos_max
        config.safety.llsafety.lim_jvel = self.state_.config.safety.llsafety.lim_jvel
        config.safety.llsafety.lim_jtor = self.state_.config.safety.llsafety.lim_jtor
        config.safety.llsafety.lim_dtor = self.state_.config.safety.llsafety.lim_dtor

        config.safety.llheartbeat.enabled = self.state_.config.safety.llheartbeat.enabled
        config.safety.llheartbeat.check_joints = self.state_.config.safety.llheartbeat.enable_CAN
        config.safety.llheartbeat.joints_timeout = self.state_.config.safety.llheartbeat.CAN_timeout
        config.safety.llheartbeat.check_IMU = self.state_.config.safety.llheartbeat.enable_IMU
        config.safety.llheartbeat.IMU_timeout = self.state_.config.safety.llheartbeat.IMU_timeout

        config.safety.torso.enabled = self.state_.config.safety.torso.enabled
        config.safety.torso.check_roll = self.state_.config.safety.torso.observe_roll
        config.safety.torso.roll_limit = self.state_.config.safety.torso.roll_limit
        config.safety.torso.check_pitch = self.state_.config.safety.torso.observe_pitch
        config.safety.torso.pitch_limit = self.state_.config.safety.torso.pitch_limit
        config.safety.torso.check_gyro = self.state_.config.safety.torso.observe_gyro
        config.safety.torso.gyro_limit = self.state_.config.safety.torso.gyro_limit