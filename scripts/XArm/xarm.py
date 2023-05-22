from typing import Optional
import numpy as np
from omni.isaac.core.robots.robot import Robot
from omni.isaac.core.prims.rigid_prim import RigidPrim
from omni.isaac.core.utils.stage import add_reference_to_stage
import pathlib


class XArm(Robot):
    """[summary]

        Args:
            prim_path (str): [description]
            name (str, optional): [description]. Defaults to "ur10_robot".
            usd_path (Optional[str], optional): [description]. Defaults to None.
            position (Optional[np.ndarray], optional): [description]. Defaults to None.
            orientation (Optional[np.ndarray], optional): [description]. Defaults to None.
    """

    def __init__(
        self,
        prim_path: str,
        name: str = "xarm_robot",
        version: Optional[int] = 7,
        position: Optional[np.ndarray] = None,
        orientation: Optional[np.ndarray] = None
    ) -> None:
        self._end_effector = None

        current_directory = str(pathlib.Path(__file__).parent.resolve()).replace("\\", "/")

        if version == 5:
            relative_usd_path = "/XArm/XArm5/xarm5.usd"
            end_link = "/link5"
        elif version == 7:
            relative_usd_path = "/XArm/XArm7/xarm7.usd"
            end_link = "/link7"

        add_reference_to_stage(usd_path=current_directory + relative_usd_path, prim_path=prim_path)

        self._end_effector_prim_path = prim_path + end_link

        super().__init__(
            prim_path=prim_path, name=name, position=position, orientation=orientation, articulation_controller=None
        )

        return

    @property
    def end_effector(self) -> RigidPrim:
        """[summary]

        Returns:
            RigidPrim: [description]
        """
        return self._end_effector

    def initialize(self, physics_sim_view=None) -> None:
        """[summary]
        """
        super().initialize(physics_sim_view)
        self._end_effector = RigidPrim(prim_path=self._end_effector_prim_path, name=self.name + "_end_effector")
        self.disable_gravity()
        self._end_effector.initialize(physics_sim_view)
        return

    def post_reset(self) -> None:
        """[summary]
        """
        Robot.post_reset(self)
        self._end_effector.post_reset()
        return
