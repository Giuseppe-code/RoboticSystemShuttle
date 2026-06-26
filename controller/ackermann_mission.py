#
# ackermann_mission.py
#

import math

from lib.system.cart import PackageTransferZone
from lib.system.controllers import PID_Controller
from lib.system.polar import Polar2DController

try:
    from .mission_config import (
        AckermannMissionConfig,
        ArmMissionPoses,
        CargoRoutingConfig,
        VisionTargetConfig,
    )
    from .mission_motion import CommandSmoother, NF1Path2DMotion
    from .mission_types import MissionCommand, MissionStateId
    from .states import (
        ArmMoveState,
        CargoArmState,
        DoneState,
        DriveToPointState,
        DriveToUnloadState,
        DropCargoArmState,
        VisionPickState,
    )
except ImportError:
    from mission_config import (
        AckermannMissionConfig,
        ArmMissionPoses,
        CargoRoutingConfig,
        VisionTargetConfig,
    )
    from mission_motion import CommandSmoother, NF1Path2DMotion
    from mission_types import MissionCommand, MissionStateId
    from states import (
        ArmMoveState,
        CargoArmState,
        DoneState,
        DriveToPointState,
        DriveToUnloadState,
        DropCargoArmState,
        VisionPickState,
    )


class AckermannMissionController:
    """State-pattern controller for the shuttle cart, cargo zones and arm."""

    STATE_CODES = {
        MissionStateId.DRIVE_TO_B: 0,
        MissionStateId.PICK_APPROACH: 1,
        MissionStateId.VISION_PICK: 2,
        MissionStateId.PICK_DOWN: 3,
        MissionStateId.PICK_LIFT_SAFE: 4,
        MissionStateId.DRIVE_TO_UNLOAD: 5,
        MissionStateId.DROP_APPROACH: 6,
        MissionStateId.DROP_DOWN: 7,
        MissionStateId.DROP_LIFT_SAFE: 8,
        MissionStateId.DONE: 9,
    }

    def __init__(
        self,
        cart,
        arm,
        config=None,
        speed_controller=None,
        position_controller=None,
        trajectory=None,
        logger=print,
        vision_callback=None,
    ):
        self.cart = cart
        self.arm = arm
        self.config = config or AckermannMissionConfig()
        self.speed_controller = speed_controller or PID_Controller(
            50.0, 10.0, 0, 100.0
        )
        self.position_controller = position_controller or Polar2DController(
            1.0, 3.0,
            5.0, math.radians(40),
        )
        self.trajectory = trajectory or NF1Path2DMotion(
            obstacles=self.config.navigation_obstacles
        )
        self.command_smoother = CommandSmoother(
            self.config.max_target_speed_rate,
            self.config.max_torque_rate,
            self.config.max_steering_rate,
        )
        self.logger = logger
        self.vision_callback = vision_callback
        self.vision_pick_pose = None
        self.vision_pick_down_pose = None
        self.last_vision_detection = None
        self.selected_cargo_color = None

        self.load_zone = PackageTransferZone(
            "B_shelf_load",
            self.config.point_b,
            self.config.zone_radius,
            PackageTransferZone.LOAD,
            self.config.packages_to_load,
        )
        self.unload_zones = {
            label: PackageTransferZone(
                label + "_unload",
                point,
                self.config.zone_radius,
                PackageTransferZone.UNLOAD,
            )
            for label, point in self.config.unload_points().items()
        }
        self.preplan_navigation_routes()
        self.cargo_events = []
        self.states = self._build_states()

        self.state_id = MissionStateId.DRIVE_TO_B
        self.command = MissionCommand()
        self.cart.set_pose(self.config.point_a[0], self.config.point_a[1])
        self.set_arm_target_or_warn(self.config.arm_poses.safe, "ARM_SAFE_POSE")
        self.states[self.state_id].enter(self)

    def _build_states(self):
        poses = self.config.arm_poses
        return {
            MissionStateId.DRIVE_TO_B: DriveToPointState(
                MissionStateId.DRIVE_TO_B,
                self.config.point_a,
                self.config.point_b,
                MissionStateId.PICK_APPROACH,
            ),
            MissionStateId.PICK_APPROACH: ArmMoveState(
                MissionStateId.PICK_APPROACH,
                poses.pick_approach,
                MissionStateId.VISION_PICK,
                "Braccio in posa di pre-scan: cerco il pacco con la camera",
            ),
            MissionStateId.VISION_PICK: VisionPickState(
                MissionStateId.PICK_DOWN,
            ),
            MissionStateId.PICK_DOWN: CargoArmState(
                MissionStateId.PICK_DOWN,
                poses.pick_down,
                MissionStateId.PICK_LIFT_SAFE,
                self.load_zone,
                "Nessun cargo_event in load: controlla posizione del carrello o zone_radius",
                "Pacco preso: porto il braccio in posizione di non ingombro",
            ),
            MissionStateId.PICK_LIFT_SAFE: ArmMoveState(
                MissionStateId.PICK_LIFT_SAFE,
                poses.safe,
                MissionStateId.DRIVE_TO_UNLOAD,
                "Braccio sicuro: scelgo la zona di scarico in base al colore",
            ),
            MissionStateId.DRIVE_TO_UNLOAD: DriveToUnloadState(
                MissionStateId.DRIVE_TO_UNLOAD,
                self.config.point_b,
                self.config.point_a,
                MissionStateId.DROP_APPROACH,
            ),
            MissionStateId.DROP_APPROACH: ArmMoveState(
                MissionStateId.DROP_APPROACH,
                poses.drop_approach,
                MissionStateId.DROP_DOWN,
                "Braccio sopra la zona di scarico: scendo",
            ),
            MissionStateId.DROP_DOWN: DropCargoArmState(
                MissionStateId.DROP_DOWN,
                poses.drop_down,
                MissionStateId.DROP_LIFT_SAFE,
                None,
                "Nessun cargo_event in unload: controlla posizione del carrello o zone_radius",
                "Pacco scaricato: riporto il braccio in posizione sicura",
            ),
            MissionStateId.DROP_LIFT_SAFE: ArmMoveState(
                MissionStateId.DROP_LIFT_SAFE,
                poses.safe,
                MissionStateId.DONE,
                "Missione completata",
            ),
            MissionStateId.DONE: DoneState(),
        }

    def log(self, message):
        if self.logger is not None:
            self.logger(message)

    def preplan_navigation_routes(self):
        if not hasattr(self.trajectory, "preplan_motion"):
            return
        for unload_point in self.config.unload_points().values():
            self.trajectory.preplan_motion(self.config.point_b, unload_point)
            self.trajectory.preplan_motion(unload_point, self.config.point_b)

    def selected_unload_label(self):
        label = self.config.unload_zone_by_color.get(
            self.selected_cargo_color,
            self.config.default_unload_zone,
        )
        if label not in self.unload_zones:
            return self.config.default_unload_zone
        return label

    def selected_unload_point(self):
        return self.config.unload_points()[self.selected_unload_label()]

    def selected_unload_zone(self):
        return self.unload_zones[self.selected_unload_label()]

    def selected_cargo_color_code(self):
        return self.config.cargo_color_code(self.selected_cargo_color)

    def selected_unload_zone_code(self):
        return self.config.unload_zone_code(self.selected_unload_label())

    def selected_unload_zone_name(self):
        return self.selected_unload_label()

    def log_arrival(self, point):
        if point == self.config.point_b:
            self.log("Arrivato al punto B: inizio presa dallo scaffale")
        elif point == self.config.point_a:
            self.log("Arrivato al punto A: preparo lo scarico")
        elif point == self.config.point_c:
            self.log("Arrivato al punto C: preparo lo scarico")

    def set_arm_target_or_warn(self, target_pose, label):
        ok = self.arm.set_target(*target_pose)
        if not ok:
            self.log("WARNING: target braccio non raggiungibile: " + str((label, target_pose)))
        return ok

    def read_vision_target(self):
        if self.vision_callback is None:
            return None
        return self.vision_callback()

    def hold_cart_command(self, delta_t):
        self.command_smoother.reset_drive_target()
        current_speed, _ = self.cart.get_speed()
        torque = self.speed_controller.evaluate(delta_t, -current_speed)
        return MissionCommand(torque=torque)

    def arm_is_near(self, target_pose):
        x_ref, y_ref, z_ref, a_ref = target_pose
        x, y, z, a = self.arm.get_pose()
        pos_error = math.sqrt(
            (x - x_ref) ** 2
            + (y - y_ref) ** 2
            + (z - z_ref) ** 2
        )
        angle_error = abs(a - a_ref)
        return (
            pos_error < self.config.arm_pos_tolerance
            and angle_error < self.config.arm_angle_tolerance
        )

    def update(self, delta_t):
        next_state_id, raw_command = self.states[self.state_id].update(self, delta_t)
        if next_state_id != self.state_id:
            self.state_id = next_state_id
            self.states[self.state_id].enter(self)
        self.command = self.command_smoother.smooth_command(delta_t, raw_command)
        return self.command

    def step(self, delta_t):
        command = self.update(delta_t)
        self.cart.evaluate(delta_t, command.torque, command.steering_angle)
        self.arm.evaluate(delta_t)
        return command

    def is_done(self):
        return self.state_id == MissionStateId.DONE

    def state_name(self):
        return self.state_id.value

    def state_code(self):
        return self.STATE_CODES[self.state_id]

    def final_report(self):
        final_pose = self.cart.get_pose_3d()
        unload_point = self.selected_unload_point()
        final_error_unload = math.hypot(
            final_pose[0] - unload_point[0],
            final_pose[1] - unload_point[1],
        )
        reached_unload = final_error_unload <= self.config.arrival_threshold
        mission_complete = (
            self.is_done()
            and reached_unload
            and self.cart.get_payload_mass() == 0
        )
        return {
            "point_a": self.config.point_a,
            "point_b": self.config.point_b,
            "point_c": self.config.point_c,
            "selected_cargo_color": self.selected_cargo_color,
            "selected_unload_zone": self.selected_unload_label(),
            "selected_unload_point": unload_point,
            "zone_radius_m": self.config.zone_radius,
            "cargo_events": self.cargo_events,
            "final_pose": final_pose,
            "final_error_from_unload_m": final_error_unload,
            "payload_kg": self.cart.get_payload_mass(),
            "reached_unload": reached_unload,
            "mission_complete": mission_complete,
        }
