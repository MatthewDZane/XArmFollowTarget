import os
import asyncio
from omni.isaac.examples.base_sample import BaseSampleExtension
from omni.isaac.examples.user_examples import XArmSample
import omni.ui as ui
from omni.isaac.ui.ui_utils import btn_builder, str_builder, state_btn_builder


class XArmExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="XArm",
            title="XArm",
            doc_link="",
            overview="",
            sample=XArmSample(),
            file_path=os.path.abspath(__file__),
            number_of_extra_frames=2,
        )
        self.task_ui_elements = {}
        frame = self.get_frame(index=0)
        self.build_task_controls_ui(frame)
        frame = self.get_frame(index=1)
        self.build_data_logging_ui(frame)
        return
    
    def _on_follow_target_button_event(self, val):
        asyncio.ensure_future(self.sample._on_follow_target_event_async(val))
        return

    def _on_add_obstacle_button_event(self):
        self.sample._on_add_obstacle_event()
        self.task_ui_elements["Remove Obstacle"].enabled = True
        return

    def _on_remove_obstacle_button_event(self):
        self.sample._on_remove_obstacle_event()
        world = self.sample.get_world()
        current_task = list(world.get_current_tasks().values())[0]
        if not current_task.obstacles_exist():
            self.task_ui_elements["Remove Obstacle"].enabled = False
        return

    def _on_logging_button_event(self, val):
        self.sample._on_logging_event(val)
        self.task_ui_elements["Save Data"].enabled = True
        return

    def _on_save_data_button_event(self):
        self.sample._on_save_data_event(self.task_ui_elements["Output Directory"].get_value_as_string())
        return

    def post_reset_button_event(self):
        self.task_ui_elements["Follow Target"].enabled = True
        self.task_ui_elements["Remove Obstacle"].enabled = False
        self.task_ui_elements["Add Obstacle"].enabled = True
        self.task_ui_elements["Start Logging"].enabled = True
        self.task_ui_elements["Save Data"].enabled = False
        if self.task_ui_elements["Follow Target"].text == "STOP":
            self.task_ui_elements["Follow Target"].text = "START"
        return

    def post_load_button_event(self):
        self.task_ui_elements["Follow Target"].enabled = True
        self.task_ui_elements["Add Obstacle"].enabled = True
        self.task_ui_elements["Start Logging"].enabled = True
        self.task_ui_elements["Save Data"].enabled = False
        return

    def post_clear_button_event(self):
        self.task_ui_elements["Follow Target"].enabled = False
        self.task_ui_elements["Remove Obstacle"].enabled = False
        self.task_ui_elements["Add Obstacle"].enabled = False
        self.task_ui_elements["Start Logging"].enabled = False
        self.task_ui_elements["Save Data"].enabled = False
        if self.task_ui_elements["Follow Target"].text == "STOP":
            self.task_ui_elements["Follow Target"].text = "START"
        return

    def shutdown_cleanup(self):
        return

    def build_task_controls_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                # Update the Frame Title
                frame.title = "Task Controls"
                frame.visible = True

                dict = {
                    "label": "Follow Target",
                    "type": "button",
                    "a_text": "START",
                    "b_text": "STOP",
                    "tooltip": "Follow Target",
                    "on_clicked_fn": self._on_follow_target_button_event,
                }
                self.task_ui_elements["Follow Target"] = state_btn_builder(**dict)
                self.task_ui_elements["Follow Target"].enabled = False

                dict = {
                    "label": "Add Obstacle",
                    "type": "button",
                    "text": "ADD",
                    "tooltip": "Add Obstacle",
                    "on_clicked_fn": self._on_add_obstacle_button_event,
                }

                self.task_ui_elements["Add Obstacle"] = btn_builder(**dict)
                self.task_ui_elements["Add Obstacle"].enabled = False
                dict = {
                    "label": "Remove Obstacle",
                    "type": "button",
                    "text": "REMOVE",
                    "tooltip": "Remove Obstacle",
                    "on_clicked_fn": self._on_remove_obstacle_button_event,
                }

                self.task_ui_elements["Remove Obstacle"] = btn_builder(**dict)
                self.task_ui_elements["Remove Obstacle"].enabled = False

    def build_data_logging_ui(self, frame):
        with frame:
            with ui.VStack(spacing=5):
                frame.title = "Data Logging"
                frame.visible = True
                dict = {
                    "label": "Output Directory",
                    "type": "stringfield",
                    "default_val": os.path.join(os.getcwd(), "output_data.json"),
                    "tooltip": "Output Directory",
                    "on_clicked_fn": None,
                    "use_folder_picker": False,
                    "read_only": False,
                }
                self.task_ui_elements["Output Directory"] = str_builder(**dict)

                dict = {
                    "label": "Start Logging",
                    "type": "button",
                    "a_text": "START",
                    "b_text": "PAUSE",
                    "tooltip": "Start Logging",
                    "on_clicked_fn": self._on_logging_button_event,
                }
                self.task_ui_elements["Start Logging"] = state_btn_builder(**dict)
                self.task_ui_elements["Start Logging"].enabled = False

                dict = {
                    "label": "Save Data",
                    "type": "button",
                    "text": "Save Data",
                    "tooltip": "Save Data",
                    "on_clicked_fn": self._on_save_data_button_event,
                }

                self.task_ui_elements["Save Data"] = btn_builder(**dict)
                self.task_ui_elements["Save Data"].enabled = False
        return
