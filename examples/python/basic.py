from time import perf_counter_ns, sleep

import trakr_sdk as sdk
from trakr_sdk import QuadDataTypes, AlliedDataTypes

ROBOT_IP = "192.168.3.50"
ROBOT_PORT = 15251

FREQ = 400
LOOP_TIME_NS = 1e9/FREQ

def main():
    robot = sdk.Robot(ROBOT_IP, ROBOT_PORT)

    plan = AlliedDataTypes.Plan()
    state = AlliedDataTypes.State()
    config = QuadDataTypes.CONFIG_SET()

    if(not robot.setup(config, plan)):
        print("[MAIN] Failed to setup robot")
        return False
    
    robot.getConfig(config)

    config_status : int = 0

    set_config : bool = False

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

        # do something here

        if(set_config):
            robot.setConfig(config)
            set_config = False

        robot.setData(plan)

        delta_t = perf_counter_ns() - time_s
        if(delta_t < LOOP_TIME_NS):
            sleep((LOOP_TIME_NS - delta_t)/1e9)

    return 0

if __name__ == "__main__":
    main()

