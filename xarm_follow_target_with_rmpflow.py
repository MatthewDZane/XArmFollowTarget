import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

from omni.isaac.kit import SimulationApp

simulation_app = SimulationApp({"headless": False})

from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.isaac.examples")
from XArm.xarm_follow_target import XArmFollowTarget
from XArm.xarm_rmpflow_controller import XArmRMPFlowController
from XArm.xarm_socket import XArmSocket
from omni.isaac.core import World
import time
import numpy as np


def main():
    world = World(stage_units_in_meters=1.0)
    xarm_task = XArmFollowTarget()
    world.add_task(xarm_task)
    world.reset()
    task_params = world.get_task("xarm_follow_target_task").get_params()
    xarm_name = task_params["robot_name"]["value"]
    target_name = task_params["target_name"]["value"]
    xarm = world.scene.get_object(xarm_name)
    cube = world.scene.get_object(target_name)
    xarm_controller = XArmRMPFlowController(name="target_follower_controller", robot_articulation=xarm)
    articulation_controller = xarm.get_articulation_controller()

    xarm_socket = XArmSocket()

    xarm_socket.start_txsocket()
    xarm_socket.start_rxsocket()

    safe_zone = [
        (0.3, -0.3, 0.3), # back bottom right 
        (0.5, 0.3, 0.625) # top front left
    ]

    start_time = time.time()
    wait_time = 1
    while simulation_app.is_running() and time.time() < start_time + wait_time:
        world.step(render=True)
        if world.is_playing():
            if world.current_time_step_index == 0:
                world.reset()
                xarm_controller.reset()
            
    while simulation_app.is_running():
        world.step(render=True)
        if world.is_playing():
            observations = world.get_observations()
            actions = xarm_controller.forward(
                target_end_effector_position=observations[task_params["target_name"]["value"]]["position"],
                target_end_effector_orientation=observations[task_params["target_name"]["value"]]["orientation"],
            )
            
            articulation_controller.set_gains(1e15*np.ones(7), 1e14*np.ones(7)) # Solution from Nvidia Live Session 1:23:00
            articulation_controller.apply_action(actions)

            if (xarm_socket.txconn):
                try: 
                    sendData = str(xarm.get_joint_positions().tolist())
                    res = xarm_socket.txconn.send(sendData.encode())
                    if res == 0:
                        print("channel is closed...")
                except:
                    # if sending failed, recreate the socket and reconnect
                    print("sending failed... closing connection")
                    xarm_socket.txconn.close()
                    xarm_socket.txconn = None

            # update position of target?
            if xarm_socket.dx and xarm_socket.dy:

                pos, _ = cube.get_world_pose()
                newpose = [ pos[0], pos[1] + xarm_socket.dx, pos[2] + xarm_socket.dy]
                newpose[1] = np.clip(newpose[1], safe_zone[0][1], safe_zone[1][1])
                newpose[2] = np.clip(newpose[2], safe_zone[0][2], safe_zone[1][2])
                print("pose", pos, "->", newpose, end="")
                cube.set_world_pose(np.array(newpose))
                print("set.")
                xarm_socket.dx = None
                xarm_socket.dy = None

    xarm_socket.shut_down_socket()
    simulation_app.close()


if __name__ == '__main__':
    main()