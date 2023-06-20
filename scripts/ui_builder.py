# This software contains source code provided by NVIDIA Corporation.
# Copyright (c) 2022-2023, NVIDIA CORPORATION.  All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto.  Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import sys
import os

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.dirname(SCRIPT_DIR))

import omni.ui as ui
import omni.timeline

from omni.isaac.core.world import World
from omni.isaac.ui.ui_utils import get_style, btn_builder, state_btn_builder, cb_builder, float_builder
from omni.usd import StageEventType

from .XArm.xarm_sample import XArmSample
import asyncio

class UIBuilder:
    def __init__(self):
        # Frames are sub-windows that can contain multiple UI elements
        self.frames = []
        # UI elements created using a UIElementWrapper instance
        self.wrapped_ui_elements = []

        self._buttons = None
        self._task_ui_elements = None

        # Get access to the timeline to control stop/pause/play programmatically
        self._timeline = omni.timeline.get_timeline_interface()

        self._sample = XArmSample()

    ###################################################################################
    #           The Functions Below Are Called Automatically By extension.py
    ###################################################################################

    def on_menu_callback(self):
        """Callback for when the UI is opened from the toolbar. 
        This is called directly after build_ui().
        """
        pass

    def on_timeline_event(self, event):
        """Callback for Timeline events (Play, Pause, Stop)

        Args:
            event (omni.timeline.TimelineEventType): Event Type
        """
        pass

    def on_physics_step(self, step: float):
        """Callback for Physics Step.
        Physics steps only occur when the timeline is playing
           
        Args:
            step (float): Size of physics step
        """
        pass

    def cleanup(self):
        """
        Called when the stage is closed or the extension is hot reloaded.
        Perform any necessary cleanup such as removing active callback functions
        Buttons imported from omni.isaac.ui.element_wrappers implement a cleanup function that should be called
        """
        if self._sample._world is not None:
            self._sample._world_cleanup()
        if self._buttons is not None:
            self._enable_all_buttons(False)
            self._buttons["Load XArm5"].enabled = True
            self._buttons["Load XArm7"].enabled = True
        self.shutdown_cleanup()
        return

    def build_ui(self):
        """
        Build a custom UI tool to run your extension.  
        This function will be called any time the UI window is closed and reopened.
        """
        self._buttons = {}
        self._task_ui_elements = {}
        world_controls_frame = ui.CollapsableFrame(title="World Controls", collapsed=False)
        with world_controls_frame:
            with ui.VStack(style=get_style(), spacing=5, height=0):
                dict = {
                    "label": "Load XArm5",
                    "type": "button",
                    "text": "Load",
                    "tooltip": "Load XArm5 and Task",
                    "on_clicked_fn": self._on_load_xarm5,
                }
                self._buttons["Load XArm5"] = btn_builder(**dict)
                self._buttons["Load XArm5"].enabled = True
                dict = {
                    "label": "Load XArm7",
                    "type": "button",
                    "text": "Load",
                    "tooltip": "Load XArm7 and Task",
                    "on_clicked_fn": self._on_load_xarm7,
                }
                self._buttons["Load XArm7"] = btn_builder(**dict)
                self._buttons["Load XArm7"].enabled = True
                dict = {
                    "label": "Reset",
                    "type": "button",
                    "text": "Reset",
                    "tooltip": "Reset robot and environment",
                    "on_clicked_fn": self._on_reset,
                }
                self._buttons["Reset"] = btn_builder(**dict)
                self._buttons["Reset"].enabled = False
        
        
        task_controls_frame = ui.CollapsableFrame(title="Task Controls", collapsed=False)
        with task_controls_frame:
            with ui.VStack(spacing=5):
                task_controls_frame.visible = True

                dict = {
                    "label": "Follow Target",
                    "type": "button",
                    "a_text": "START",
                    "b_text": "STOP",
                    "tooltip": "Follow Target",
                    "on_clicked_fn": self._on_follow_target_button_event,
                }
                self._task_ui_elements["Follow Target"] = state_btn_builder(**dict)
                self._task_ui_elements["Follow Target"].enabled = False

                dict = {
                    "label": "Add Obstacle",
                    "type": "button",
                    "text": "ADD",
                    "tooltip": "Add Obstacle",
                    "on_clicked_fn": self._on_add_obstacle_button_event,
                }

                self._task_ui_elements["Add Obstacle"] = btn_builder(**dict)
                self._task_ui_elements["Add Obstacle"].enabled = False
                dict = {
                    "label": "Remove Obstacle",
                    "type": "button",
                    "text": "REMOVE",
                    "tooltip": "Remove Obstacle",
                    "on_clicked_fn": self._on_remove_obstacle_button_event,
                }

                self._task_ui_elements["Remove Obstacle"] = btn_builder(**dict)
                self._task_ui_elements["Remove Obstacle"].enabled = False

                dict = {
                    "label": "Random Target Enabled",
                    "type": "checkbox",
                    "default_val": True,
                    "tooltip": "Random Target Enabled",
                    "on_clicked_fn": self._on_random_target_enabled_event
                }
                self._task_ui_elements["Random Target Checkbox"] = cb_builder(**dict)

                args = {
                    "label": "Min Face Range",
                    "default_val": .3,
                    "tooltip": "Min Range in Meters",
                }
                self._task_ui_elements["Min Face Range"] = float_builder(**args)
                self._task_ui_elements["Min Face Range"].add_value_changed_fn(self._on_min_face_range_changed_event)

                args = {
                    "label": "Max Face Range",
                    "default_val": 10,
                    "tooltip": "Max Range in Meters",
                }
                self._task_ui_elements["Max Face Range"] = float_builder(**args)
                self._task_ui_elements["Max Face Range"].add_value_changed_fn(self._on_max_face_range_changed_event)


    def _on_load_xarm5(self):
        self._sample.set_xarm_version(5)
        self._on_random_target_enabled_event(False)
        self._on_min_face_range_changed_event(self._task_ui_elements["Min Face Range"].get_value_as_float())
        self._on_max_face_range_changed_event(self._task_ui_elements["Max Face Range"].get_value_as_float())


        async def _on_load_world_async():
            await self._sample.load_world_async()
            await omni.kit.app.get_app().next_update_async()
            self._sample._world.add_stage_callback("stage_event_1", self.on_stage_event)
            self._enable_all_buttons(True)
            self._buttons["Load XArm5"].enabled = False
            self._buttons["Load XArm7"].enabled = False
            self.post_load_button_event()
            self._sample._world.add_timeline_callback("stop_reset_event", self._reset_on_stop_event)

        asyncio.ensure_future(_on_load_world_async())
        return
    
    def _on_load_xarm7(self):
        self._sample.set_xarm_version(7)
        self._on_random_target_enabled_event(False)
        self._on_min_face_range_changed_event(self._task_ui_elements["Min Face Range"].get_value_as_float())
        self._on_max_face_range_changed_event(self._task_ui_elements["Max Face Range"].get_value_as_float())


        async def _on_load_world_async():
            await self._sample.load_world_async()
            await omni.kit.app.get_app().next_update_async()
            self._sample._world.add_stage_callback("stage_event_1", self.on_stage_event)
            self._enable_all_buttons(True)
            self._buttons["Load XArm7"].enabled = False
            self._buttons["Load XArm5"].enabled = False
            self.post_load_button_event()
            self._sample._world.add_timeline_callback("stop_reset_event", self._reset_on_stop_event)

        asyncio.ensure_future(_on_load_world_async())
        return

    def _on_reset(self):
        async def _on_reset_async():
            await self._sample.reset_async()
            await omni.kit.app.get_app().next_update_async()
            self.post_reset_button_event()

        asyncio.ensure_future(_on_reset_async())
        return

    def _on_follow_target_button_event(self, val):
        asyncio.ensure_future(self._sample._on_follow_target_event_async(val))
        return

    def _on_add_obstacle_button_event(self):
        self._sample._on_add_obstacle_event()
        self._task_ui_elements["Remove Obstacle"].enabled = True
        return

    def _on_remove_obstacle_button_event(self):
        self._sample._on_remove_obstacle_event()
        world = self._sample.get_world()
        current_task = list(world.get_current_tasks().values())[0]
        if not current_task.obstacles_exist():
            self._task_ui_elements["Remove Obstacle"].enabled = False
        return
    
    def _on_random_target_enabled_event(self, val):
        self._sample.rand_target_enabled = self._task_ui_elements["Random Target Checkbox"].get_value_as_bool()

    def _on_min_face_range_changed_event(self, val):
        self._sample.min_detection_range = self._task_ui_elements["Min Face Range"].get_value_as_float()

    def _on_max_face_range_changed_event(self, val):
        self._sample.max_detection_range = self._task_ui_elements["Max Face Range"].get_value_as_float()
    
    def _enable_all_buttons(self, flag):
        for btn_name, btn in self._buttons.items():
            if isinstance(btn, omni.ui._ui.Button):
                btn.enabled = flag
        return
    def _on_follow_target_button_event(self, val):
        asyncio.ensure_future(self._sample._on_follow_target_event_async(val))
        return


    def _on_save_data_button_event(self):
        self._sample._on_save_data_event(self._task_ui_elements["Output Directory"].get_value_as_string())
        return

    def post_reset_button_event(self):
        self._task_ui_elements["Follow Target"].enabled = True
        self._task_ui_elements["Remove Obstacle"].enabled = False
        self._task_ui_elements["Add Obstacle"].enabled = True
        if self._task_ui_elements["Follow Target"].text == "STOP":
            self._task_ui_elements["Follow Target"].text = "START"
        return

    def post_load_button_event(self):
        self._task_ui_elements["Follow Target"].enabled = True
        self._task_ui_elements["Add Obstacle"].enabled = True
        return

    def post_clear_button_event(self):
        self._task_ui_elements["Follow Target"].enabled = False
        self._task_ui_elements["Remove Obstacle"].enabled = False
        self._task_ui_elements["Add Obstacle"].enabled = False
        if self._task_ui_elements["Follow Target"].text == "STOP":
            self._task_ui_elements["Follow Target"].text = "START"
        return
    
    def shutdown_cleanup(self):
        return

    def on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED):
            # If the user opens a new stage, the extension should completely reset
            self._task_ui_elements["Follow Target"].enabled = False
            self._task_ui_elements["Remove Obstacle"].enabled = False
            self._task_ui_elements["Add Obstacle"].enabled = False
            if self._task_ui_elements["Follow Target"].text == "STOP":
                self._task_ui_elements["Follow Target"].text = "START"
        elif event.type == int(omni.usd.StageEventType.CLOSED):
            if World.instance() is not None:
                self._sample._world_cleanup()
                self._sample._world.clear_instance()
                if hasattr(self, "_buttons"):
                    if self._buttons is not None:
                        self._enable_all_buttons(False)
                        self._buttons["Load XArm5"].enabled = True
                        self._buttons["Load XArm7"].enabled = True
        return
    
    def _reset_on_stop_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            #self._buttons["Load XArm5"].enabled = False
            #self._buttons["Load XArm7"].enabled = False
            self._buttons["Reset"].enabled = True
            self.post_clear_button_event()
        return 
