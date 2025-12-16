from enum import Enum

# MotionDataTypes
class TaskTypes(Enum):
    eGesture = 0
    eMotion = 1

class MotionModes(Enum):
    eNone = 0
    eClassicalMode = 1
    eAIMode = 2
    eDeveloperMode = 3

class MotionGaits(Enum):
    eStance = 0
    eTrot = 1

class GestureTypes(Enum):
    eNone = 0
    eStandUp = 1
    eSitDown = 2
    eOrientation = 3
    eLeftHandShake = 4
    eRightHandShake = 5
    eReserved = 6

class BaseConfig:
    type : int
    seq : int

class MotionConfig(BaseConfig):
    type : MotionModes = MotionModes.eClassicalMode
    seq : MotionGaits = MotionGaits.eStance

class GestureConfig(BaseConfig):
    type : GestureTypes = GestureTypes.eStandUp

class CONFIG_SET:
    planner : TaskTypes = TaskTypes.eGesture

    sequence : GestureConfig
    strategy : MotionConfig

    def __init__(self):
        self.sequence = GestureConfig()
        self.strategy = MotionConfig()