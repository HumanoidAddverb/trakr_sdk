from enum import Enum

from . import motion_configs as MotionDataTypes
from . import safety_configs as SafetyDataTypes

class ROBOT_CONFIG:
    shutdown : bool = False
    exit : bool = False
    killed : bool = False

    def doExit(self):
        self.shutdown = True
        self.exit = True

    def doShutDown(self):
        self.shutdown = True

    def kill(self):
        self.killed = True

class CONFIG_SET:
    master : ROBOT_CONFIG
    motion : MotionDataTypes.CONFIG_SET
    safety : SafetyDataTypes.CONFIG_SET

    def __init__(self):
        self.master = ROBOT_CONFIG()
        self.motion = MotionDataTypes.CONFIG_SET()
        self.safety = SafetyDataTypes.CONFIG_SET()