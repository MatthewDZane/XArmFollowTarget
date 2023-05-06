from omni.isaac.examples.base_sample import BaseSample
from omni.isaac.examples.user_examples.XArm.xarm_follow_target import XArmFollowTarget
from omni.isaac.examples.user_examples.XArm.xarm_rmpflow_controller import XArmRMPFlowController
import numpy as np
import ast
import threading
import socket
import carb

class XArmSample(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._controller = None
        self._articulation_controller = None
        # sending position data to arm
        self._txsocket = None
        self._txconn = None
        
        # tracking info
        self._rxsocket = None
        self._rxconn = None

        self._safe_zone = [
            (0.3, -0.3, 0.3), # back bottom right 
            (0.5, 0.3, 0.625) # top front left
                           ]
        
        self._dx = None
        self._dy = None


    def setup_scene(self):
        world = self.get_world()
        world.add_task(XArmFollowTarget(world))
        return
    
    def _setup_txsocket(self):
        if self._txsocket is None:
            self._txsocket = socket.socket()
            # allow socket to reuse address
            self._txsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            txport = 12345
            self._txsocket.bind(('', txport))
        
        # https://docs.python.org/3/library/socket.html#socket.socket.listen
        self._txsocket.listen(5) # number of unaccepted connections allow (backlog)
        
        while True:
            self._txconn, self._txaddr = self._txsocket.accept()
            print("accepted tx connection from:",str(self._txaddr[0]), ":", str(self._txaddr[1]))

    def _setup_rxsocket(self):
        if self._rxsocket is None:
            self._rxsocket = socket.socket()
            # allow socket to reuse address
            self._rxsocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            rxport = 12346
            self._rxsocket.bind(('', rxport))
        
        # https://docs.python.org/3/library/socket.html#socket.socket.listen
        self._rxsocket.listen(5) # number of unaccepted connections allow (backlog)
        
        while True:
            self._rxconn, self._rxaddr = self._rxsocket.accept()
            print("accepted rx connection from:",str(self._rxaddr[0]), ":", str(self._rxaddr[1]))

            while True:
                data = self._rxconn.recv(1024)
                if data:
                    message = data.decode()
                    # print("received:", type(message), message)
                    x, y, z, dx, dy = ast.literal_eval(message)
                    # print("received:", x, y, z, dx, dy)
                    weight = 0.1
                    self._dx = weight*dx
                    self._dy = weight*dy
                else:
                    break
                # else:
                    # break
                # didn't receive anything


    def _shut_down_socket(self):
        if self._txconn:
            try:
                # self._conn.send("Done".encode())
                self._txsocket.shutdown(socket.SHUT_RDWR)
                self._txsocket.close()
                self._txsocket = None
            except socket.error as e:
                pass
        if self._rxconn:
            try:
                # self._conn.send("Done".encode())
                self._rxsocket.shutdown(socket.SHUT_RDWR)
                self._rxsocket.close()
                self._rxsocket = None
            except socket.error as e:
                pass

    async def setup_pre_reset(self):
        world = self.get_world()
        if world.physics_callback_exists("sim_step"):
            world.remove_physics_callback("sim_step")
        self._controller.reset()
        return

    def world_cleanup(self):
        self._controller = None
        self._shut_down_socket()
        return

    async def setup_post_load(self):
        self._xarm_task = list(self._world.get_current_tasks().values())[0]
        self._task_params = self._xarm_task.get_params()
        self._xarm = self._world.scene.get_object(self._task_params["robot_name"]["value"])
        self._controller = XArmRMPFlowController(name="target_follower_controller", robot_articulation=self._xarm)
        self._articulation_controller = self._xarm.get_articulation_controller()

        self._txsocket_thread = threading.Thread(target=self._setup_txsocket)
        self._txsocket_thread.start()

        self._rxsocket_thread = threading.Thread(target=self._setup_rxsocket)
        self._rxsocket_thread.start()
        return

    async def _on_follow_target_event_async(self, val):
        world = self.get_world()
        if val:
            await world.play_async()
            world.add_physics_callback("sim_step", self._on_follow_target_simulation_step)
        else:
            world.remove_physics_callback("sim_step")
        return

    def _on_follow_target_simulation_step(self, step_size):
        observations = self._world.get_observations()
        actions = self._controller.forward(
            target_end_effector_position=observations[self._task_params["target_name"]["value"]]["position"],
            target_end_effector_orientation=observations[self._task_params["target_name"]["value"]]["orientation"],
        )
        
        self._articulation_controller.set_gains(1e15*np.ones(7), 1e14*np.ones(7)) # Solution from Nvidia Live Session 1:23:00
        self._articulation_controller.apply_action(actions)

        if (self._txconn):
            try: 
                sendData = str(self._xarm.get_joint_positions().tolist())
                res = self._txconn.send(sendData.encode())
                if res == 0:
                    print("channel is closed...")
            except:
                # if sending failed, recreate the socket and reconnect
                print("sending failed... closing connection")
                self._txconn.close()
                self._txconn = None

        # update position of target?
        if self._dx and self._dy:
            world = self.get_world()
            cube = world.scene.get_object("target")
            pos, _ = cube.get_world_pose()
            newpose = [ pos[0], pos[1]+self._dx, pos[2]+self._dy]
            newpose[1] = np.clip(newpose[1], self._safe_zone[0][1], self._safe_zone[1][1])
            newpose[2] = np.clip(newpose[2], self._safe_zone[0][2], self._safe_zone[1][2])
            print("pose", pos, "->", newpose, end="")
            cube.set_world_pose(np.array(newpose))
            print("set.")
            self._dx = None
            self._dy = None

        # if (self._rxconn):
        #     try:
        #         data = self._rxconn.recv(1024, 0x40) # non-blocking receive
        #         message = data.decode()
        #         print("received:", type(message), message)
        #         x, y, z, dx, dy = ast.literal_eval(message)
        #         print("received:", x, y, z, dx, dy)
        #         world = self.get_world()
        #         cube = world.get_object("target")
        #         pos, _ = cube.get_world_pose()
        #         newpose = [ pos[0], pos[1]+dy, pos[2]+dx]
        #         newpose[1] = np.clip(newpose[1], self._safe_zone[0][1], self._safe_zone[1][1])
        #         newpose[2] = np.clip(newpose[2], self._safe_zone[0][2], self._safe_zone[1][2])
        #         print("newpose:", newpose)
        #         cube.set_world_pose(np.array(newpose))
        #     except:
        #         # didn't receive anything
                
        #         pass

        return

    def _on_add_obstacle_event(self):
        world = self.get_world()
        current_task = list(world.get_current_tasks().values())[0]
        cube = current_task.add_obstacle()
        self._controller.add_obstacle(cube)
        return

    def _on_remove_obstacle_event(self):
        world = self.get_world()
        current_task = list(world.get_current_tasks().values())[0]
        obstacle_to_delete = current_task.get_obstacle_to_delete()
        self._controller.remove_obstacle(obstacle_to_delete)
        current_task.remove_obstacle()
        return

    def _on_logging_event(self, val):
        world = self.get_world()
        data_logger = world.get_data_logger()
        if not world.get_data_logger().is_started():
            robot_name = self._task_params["robot_name"]["value"]
            target_name = self._task_params["target_name"]["value"]

            def frame_logging_func(tasks, scene):
                return {
                    "joint_positions": scene.get_object(robot_name).get_joint_positions().tolist(),
                    "applied_joint_positions": scene.get_object(robot_name)
                    .get_applied_action()
                    .joint_positions.tolist(),
                    "target_position": scene.get_object(target_name).get_world_pose()[0].tolist(),
                }

            data_logger.add_data_frame_logging_func(frame_logging_func)
        if val:
            data_logger.start()
        else:
            data_logger.pause()
        return

    def _on_save_data_event(self, log_path):
        world = self.get_world()
        data_logger = world.get_data_logger()
        data_logger.save(log_path=log_path)
        data_logger.reset()
        return
