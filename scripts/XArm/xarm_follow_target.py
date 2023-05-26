from .xarm import XArm
import numpy as np
import omni.isaac.core.tasks as tasks
from omni.isaac.core.utils.stage import get_stage_units
import carb


class XArmFollowTarget(tasks.FollowTarget):
    """[summary]

        Args:
            name (str, optional): [description]. Defaults to "ur10_follow_target".
            target_prim_path (Optional[str], optional): [description]. Defaults to None.
            target_name (Optional[str], optional): [description]. Defaults to None.
            target_position (Optional[np.ndarray], optional): [description]. Defaults to None.
            target_orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
        """

    def __init__(self, xarm_version: int = 7):
        super().__init__(name="xarm_follow_target_task", target_position=np.array([0.3, 0.0, 0.5]) / get_stage_units(), offset=None)
        self._goal_position = np.array([0, 0, 0])
        self.task_achieved = False
        self.xarm_version = xarm_version
        return
    
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        self._cube = scene.get_object("target")
        # cpose, crot = self._cube.get_world_pose()
        # self._cube.set_world_pose(cpose, np.Array([180.0, 90, -180]))
        return
    
    def pre_step(self, control_index, simulation_time):
        self._goal_position, orient = self._cube.get_world_pose()
        end_effector_position, _ = self._xarm.end_effector.get_world_pose()
        # print("orientation"+str(orient))
        dist = np.mean(np.abs(self._goal_position - end_effector_position))
        if not self.task_achieved is bool(dist < 0.02):
            self.task_achieved = bool(dist < 0.02)
            if self.task_achieved:
                print("Target Reached")
                self._cube.get_applied_visual_material().set_color(color=np.array([0, 1.0, 0]))
            else:
                self._cube.get_applied_visual_material().set_color(color=np.array([1.0, 0, 0]))
        return
    

    def set_robot(self) -> XArm:
        """[summary]

        Returns:
            XArm: [description]
        """
        if self.xarm_version == 5:
            prim_path = "/World/XArm5"
            name = "xarm5"
            positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0])
        elif self.xarm_version == 7:
            prim_path = "/World/XArm7"
            name = "xarm7"
            positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0])

        self._xarm = XArm(prim_path=prim_path, name=name, version=self.xarm_version)

        self._xarm.set_joints_default_state(
            positions=positions
        )
        return self._xarm
