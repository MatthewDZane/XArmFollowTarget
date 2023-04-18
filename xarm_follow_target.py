from omni.isaac.examples.user_examples.XArm.xarm import XArm
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

    def __init__(self, name):
        super().__init__(name=name, target_position=np.array([0.3, 0.0, 0.5]) / get_stage_units(), offset=None)
        self._goal_position = np.array([0, 0, 0])
        self._task_achieved = False
        return
    
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        self._cube = scene.get_object("target")
        return
    
    def pre_step(self, control_index, simulation_time):
        self._goal_position, _ = self._cube.get_world_pose()
        end_effector_position, _ = self._xarm.end_effector.get_world_pose()

        dist = np.mean(np.abs(self._goal_position - end_effector_position))
        if not self._task_achieved is bool(dist < 0.02):
            self._task_achieved = bool(dist < 0.02)
            if self._task_achieved:
                carb.log_info("Target Reached")
                self._cube.get_applied_visual_material().set_color(color=np.array([0, 1.0, 0]))
            else:
                self._cube.get_applied_visual_material().set_color(color=np.array([0, 0, 1.0]))
        return
    

    def set_robot(self) -> XArm:
        """[summary]

        Returns:
            XArm: [description]
        """
        self._xarm = XArm(prim_path="/World/XArm", name="xarm")

        self._xarm.set_joints_default_state(
            positions=np.array([-np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2, 0])
        )
        return self._xarm
