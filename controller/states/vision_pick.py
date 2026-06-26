import math

try:
    from ..mission_types import MissionCommand, MissionStateId
    from .base import MissionState
except ImportError:
    from mission_types import MissionCommand, MissionStateId
    from states.base import MissionState


class VisionPickState(MissionState):
    def __init__(self, next_state):
        self.state_id = MissionStateId.VISION_PICK
        self.next_state = next_state
        self.scan_index = 0
        self.scan_timer = 0.0
        self.sample_timer = 0.0
        self.lock_count = 0
        self.warned_missing_provider = False
        self.mode = "scan"
        self.best_detection = None
        self.best_pose = None
        self.entry_pose = None
        self.approach_start_pose = None
        self.track_pose_timer = 0.0

    def enter(self, mission):
        self.scan_index = 0
        self.scan_timer = 0.0
        self.sample_timer = 0.0
        self.lock_count = 0
        self.mode = "scan"
        self.best_detection = None
        self.best_pose = None
        self.entry_pose = mission.cart.get_pose()
        self.approach_start_pose = None
        self.track_pose_timer = 0.0
        mission.vision_pick_pose = None
        mission.vision_pick_down_pose = None
        self._set_scan_pose(mission)

    def update(self, mission, delta_t):
        command = mission.hold_cart_command(delta_t)
        if not mission.config.vision_pick_enabled:
            return self.next_state, command

        if mission.vision_callback is None:
            if not self.warned_missing_provider:
                mission.log("Vision provider assente: procedo con pick nominale")
                self.warned_missing_provider = True
            return self.next_state, command

        if self.mode == "approach":
            return self.state_id, self._approach_cart(mission, delta_t)

        self.scan_timer += delta_t
        self.sample_timer += delta_t
        if self.sample_timer < mission.config.vision_sample_period:
            return self.state_id, command
        self.sample_timer = 0.0

        if (
            self.mode == "track"
            and self.best_pose is not None
            and not mission.arm_is_near(self.best_pose)
        ):
            self.track_pose_timer += delta_t
            if self.track_pose_timer < mission.config.vision_track_pose_timeout:
                return self.state_id, command
            mission.log("Vision best pose non raggiunta: continuo il tracking dalla posa corrente")
        self.track_pose_timer = 0.0

        detection = mission.read_vision_target()
        if self.mode == "scan":
            if detection is not None:
                area = detection.get("area", 0)
        
                # aggiorna sempre il best
                best_area = self.best_detection.get("area", 0) if self.best_detection else 0
                if area > best_area:
                    self.best_detection = detection
                    self.best_pose = mission.arm.get_pose()
                    mission.last_vision_detection = detection


                # Fix 3: se trovi subito qualcosa di buono, passa a track senza aspettare il giro completo
                if area > mission.config.vision_min_track_area:
                    if self._is_trackable(mission, detection):
                        self.best_detection = detection
                        self.best_pose = mission.arm.get_pose()
                        mission.last_vision_detection = detection
                        self.mode = "track"
                        self.lock_count = 0
                        self.track_pose_timer = 0.0
                        return self.state_id, command

            self._advance_scan_if_ready(mission)
            return self.state_id, command

        if detection is None:
            self.lock_count = 0
            self._restart_scan(mission)
            return self.state_id, command
        if not self._is_inside_track_window(mission, detection):
            self.lock_count = 0
            if self._can_approach_cart(mission):
                self.mode = "approach"
                self.approach_start_pose = mission.cart.get_pose()
                mission.log("Vision target al bordo: avanzo lentamente verso lo scaffale")
                return self.state_id, command
            self._restart_scan(mission)
            return self.state_id, command

        cx = detection["cx"]
        cy = detection["cy"]
        center_x, center_y = mission.config.vision_center
        print(f"[DEBUG] center_x={center_x} center_y={center_y} cx={cx} cy={cy}")
        err_x = center_x - cx
        err_y = center_y - cy
        print(f"[DEBUG] err_x={err_x} err_y={err_y}")
        mission.last_vision_detection = detection

        if (
            abs(err_x) <= mission.config.vision_pixel_tolerance
            and abs(err_y) <= mission.config.vision_pixel_tolerance
        ):
            self.lock_count += 1
            if self.lock_count >= mission.config.vision_lock_frames:
                x, y, z, a = mission.arm.get_pose()
                mission.vision_pick_pose = (x, y, z, a)
                mission.vision_pick_down_pose = (
                    x,
                    y,
                    mission.config.arm_poses.pick_down[2],
                    a,
                )
                mission.selected_cargo_color = detection.get("color")
                mission.log(
                    "Vision target locked: "
                    + str({
                        "color": mission.selected_cargo_color,
                        "cx": cx,
                        "cy": cy,
                        "area": detection.get("area"),
                    })
                )
                return self.next_state, command
            return self.state_id, command

        self.lock_count = 0
        x, y, z, a = mission.arm.get_pose()
        gain = mission.config.vision_pixel_to_arm_gain
        max_step = mission.config.vision_max_adjust_step
        step_x = self._clamp(-err_y * gain, -max_step, max_step)
        step_y = self._clamp(-err_x * gain, -max_step, max_step)
        target = (x + step_x, y + step_y, z, a)
        print(f"[track] cy={cy} err_y={err_y:.1f} step_x={step_x:.4f} → braccio {'avanza' if step_x > 0 else 'arretra'}")
        if not mission.set_arm_target_or_warn(target, "VISION_PICK_TRACK"):
            self._advance_scan_if_ready(mission, force=True)
        return self.state_id, command

    def _set_scan_pose(self, mission):
        poses = mission.config.vision_scan_poses
        if not poses:
            mission.set_arm_target_or_warn(
                mission.config.arm_poses.pick_approach,
                "VISION_PICK_SCAN",
            )
            return
        pose = poses[self.scan_index % len(poses)]
        mission.set_arm_target_or_warn(pose, "VISION_PICK_SCAN")

    def _advance_scan_if_ready(self, mission, force=False):
        if not force and self.scan_timer < mission.config.vision_scan_dwell:
            return
        self.scan_timer = 0.0
        self.scan_index += 1
        poses = mission.config.vision_scan_poses
        if poses and self.scan_index >= len(poses):
            if self.best_detection is not None and self.best_pose is not None:
                mission.log(
                    "Vision 360 scan best target: "
                    + str({
                        "color": self.best_detection.get("color"),
                        "cx": self.best_detection.get("cx"),
                        "cy": self.best_detection.get("cy"),
                        "area": self.best_detection.get("area"),
                    })
                )
                if self._is_trackable(mission, self.best_detection):
                    self.mode = "track"
                    self.lock_count = 0
                    self.track_pose_timer = 0.0
                    mission.set_arm_target_or_warn(self.best_pose, "VISION_PICK_BEST_POSE")
                    return
                if self._can_approach_cart(mission):
                    self.mode = "approach"
                    self.approach_start_pose = mission.cart.get_pose()
                    mission.log("Vision target lontano o al bordo: avanzo lentamente verso lo scaffale")
                    return
            if self._can_approach_cart(mission):
                self.mode = "approach"
                self.approach_start_pose = mission.cart.get_pose()
                mission.log("Vision scan senza target stabile: avanzo lentamente verso lo scaffale")
                return
            self.scan_index = 0
        self._set_scan_pose(mission)

    def _restart_scan(self, mission):
        self.mode = "scan"
        self.lock_count = 0
        self.scan_index = 0
        self.scan_timer = 0.0
        self.track_pose_timer = 0.0
        self.best_detection = None
        self.best_pose = None
        self._set_scan_pose(mission)

    def _can_approach_cart(self, mission):
        if not mission.config.vision_cart_approach_enabled:
            return False
        if self.entry_pose is None:
            return False
        current = mission.cart.get_pose()
        approached = math.hypot(
            current[0] - self.entry_pose[0],
            current[1] - self.entry_pose[1],
        )
        return approached < mission.config.vision_cart_max_approach

    def _is_trackable(self, mission, detection):
        if detection is None:
            return False
        if detection.get("area", 0) < mission.config.vision_min_track_area:
            return False
        return self._is_inside_track_window(mission, detection)

    def _is_inside_track_window(self, mission, detection):
        cx = detection.get("cx", -1)
        cy = detection.get("cy", -1)
        margin = mission.config.vision_track_margin_px
        max_x = mission.config.vision_center[0] * 2
        max_y = mission.config.vision_center[1] * 2
        return margin <= cx <= max_x - margin and margin <= cy <= max_y - margin

    def _approach_cart(self, mission, delta_t):
        current = mission.cart.get_pose()
        current_speed, _ = mission.cart.get_speed()
        if self.approach_start_pose is None:
            self.approach_start_pose = current
        step_distance = math.hypot(
            current[0] - self.approach_start_pose[0],
            current[1] - self.approach_start_pose[1],
        )
        total_distance = 0.0
        if self.entry_pose is not None:
            total_distance = math.hypot(
                current[0] - self.entry_pose[0],
                current[1] - self.entry_pose[1],
            )
        if (
            step_distance >= mission.config.vision_cart_approach_step
            or total_distance >= mission.config.vision_cart_max_approach
        ):
            self._restart_scan(mission)
            return mission.hold_cart_command(delta_t)

        target_speed = mission.config.vision_cart_approach_speed
        torque = mission.speed_controller.evaluate(delta_t, target_speed - current_speed)
        torque = mission.command_smoother.evaluate_torque(delta_t, torque)
        return MissionCommand(
            torque=torque,
            steering_angle=0.0,
            target_speed=target_speed,
        )

    @staticmethod
    def _clamp(value, lower, upper):
        return max(lower, min(upper, value))
