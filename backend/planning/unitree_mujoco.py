import time
import mujoco
import mujoco.viewer
from threading import Thread
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py_bridge import UnitreeSdk2Bridge

import config


locker = threading.Lock()

mj_model = mujoco.MjModel.from_xml_path(config.ROBOT_SCENE)
mj_data = mujoco.MjData(mj_model)

viewer = mujoco.viewer.launch_passive(mj_model, mj_data)

mj_model.opt.timestep = config.SIMULATE_DT
num_motor_ = mj_model.nu
dim_motor_sensor_ = 3 * num_motor_

time.sleep(0.2)


def SimulationThread():
    global mj_data, mj_model

    # Comment out the bridge to disable command processing
    # ChannelFactoryInitialize(config.DOMAIN_ID, config.INTERFACE)
    # unitree = UnitreeSdk2Bridge(mj_model, mj_data)

    # if config.USE_JOYSTICK:
    #     unitree.SetupJoystick(device_id=0, js_type=config.JOYSTICK_TYPE)
    # if config.PRINT_SCENE_INFORMATION:
    #     unitree.PrintSceneInformation()

    # Set all motor controls to zero (no actuation)
    for i in range(num_motor_):
        mj_data.ctrl[i] = 0.0

    print(f"Running simulation with {num_motor_} motors")
    print("Bridge disabled - robot should fall naturally under gravity")

    while viewer.is_running():
        step_start = time.perf_counter()

        locker.acquire()

        # Keep all controls at zero (no actuation)
        for i in range(num_motor_):
            mj_data.ctrl[i] = 0.0

        mujoco.mj_step(mj_model, mj_data)

        locker.release()

        time_until_next_step = mj_model.opt.timestep - (
            time.perf_counter() - step_start
        )
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)


def PhysicsViewerThread():
    while viewer.is_running():
        locker.acquire()
        viewer.sync()
        locker.release()
        time.sleep(config.VIEWER_DT)


if __name__ == "__main__":
    viewer_thread = Thread(target=PhysicsViewerThread)
    sim_thread = Thread(target=SimulationThread)

    viewer_thread.start()
    sim_thread.start()
