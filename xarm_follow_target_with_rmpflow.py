import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.examples")
from .xarm_follow_target import XArmFollowTarget
from .xarm_rmpflow_controller import XArmRMPFlowController
from omni.isaac.core import World
import time

def main():
    my_world = World(stage_units_in_meters=1.0)
    my_task = XArmFollowTarget(name="xarm_follow_target_task")
    my_world.add_task(my_task)
    my_world.reset()
    task_params = my_world.get_task("xarm_follow_target_task").get_params()
    xarm_name = task_params["robot_name"]["value"]
    target_name = task_params["target_name"]["value"]
    xarm_name = my_world.scene.get_object(xarm_name)
    my_controller = XArmRMPFlowController(name="target_follower_controller", robot_articulation=xarm_name)
    articulation_controller = xarm_name.get_articulation_controller()

    start_time = time.time()
    wait_time = 1
    while simulation_app.is_running() and time.time() < start_time + wait_time:
        my_world.step(render=True)
        if my_world.is_playing():
            if my_world.current_time_step_index == 0:
                my_world.reset()
                my_controller.reset()
            
    while simulation_app.is_running():
        my_world.step(render=True)
        if my_world.is_playing():
            observations = my_world.get_observations()
            actions = my_controller.forward(
                target_end_effector_position=observations[target_name]["position"],
                target_end_effector_orientation=observations[target_name]["orientation"],
            )
            articulation_controller.apply_action(actions)

    simulation_app.close()


if __name__ == '__main__':
    main()