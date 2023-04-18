import omni.isaac.motion_generation as mg
from omni.isaac.core.articulations import Articulation


class XArmRMPFlowController(mg.MotionPolicyController):
    """[summary]

        Args:
            name (str): [description]
            robot_articulation (Articulation): [description]
            physics_dt (float, optional): [description]. Defaults to 1.0/60.0.
            attach_gripper (bool, optional): [description]. Defaults to False.
        """

    def __init__(
        self, name: str, robot_articulation: Articulation, physics_dt: float = 1.0 / 60.0
    ) -> None:

        self.rmp_flow = mg.lula.motion_policies.RmpFlow(
            robot_description_path="C:/users/matth/appdata/local/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/user_examples/XArm/XArm/xarm_descriptor.yaml",
            urdf_path="C:/users/matth/appdata/local/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/user_examples/XArm/XArm/xarm.urdf",
            rmpflow_config_path="C:/users/matth/appdata/local/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.examples/omni/isaac/examples/user_examples/XArm/XArm/xarm_rmpflow_common.yaml",
            end_effector_frame_name="link7",
            maximum_substep_size=0.00334,
            ignore_robot_state_updates=False,
        )

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmp_flow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        self._default_position, self._default_orientation = (
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
