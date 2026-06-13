#
# ackermann_mission.py
#

from dataclasses import dataclass, field
from enum import Enum
import math

from lib.system.cart import PackageTransferZone
from lib.system.controllers import PID_Controller
from lib.system.polar import Polar2DController

try:
    from .nf1 import NF1Planner
    from .world import World
except ImportError:
    from nf1 import NF1Planner
    from world import World


class MissionStateId(Enum):
    DRIVE_TO_B = "DRIVE_TO_B"
    PICK_APPROACH = "PICK_APPROACH"
    PICK_DOWN = "PICK_DOWN"
    PICK_LIFT_SAFE = "PICK_LIFT_SAFE"
    DRIVE_TO_A = "DRIVE_TO_A"
    DROP_APPROACH = "DROP_APPROACH"
    DROP_DOWN = "DROP_DOWN"
    DROP_LIFT_SAFE = "DROP_LIFT_SAFE"
    DONE = "DONE"


@dataclass
class ArmMissionPoses:
    safe: tuple = (0.20, 0.00, 0.55, math.radians(-90))
    pick_approach: tuple = (0.25, 0.45, 0.40, math.radians(-90))
    pick_down: tuple = (0.25, 0.45, 0.12, math.radians(-90))
    drop_approach: tuple = (0.25, 0.00, 0.35, math.radians(-90))
    drop_down: tuple = (0.25, 0.00, 0.08, math.radians(-90))


@dataclass
class AckermannMissionConfig:
    point_a: tuple = (0.0, 0.0)
    point_b: tuple = (0.0, -25.0)
    zone_radius: float = 0.35
    packages_to_load: list = field(default_factory=lambda: [5.0, 5.0, 5.0])
    arrival_threshold: float | None = None
    arrival_speed_threshold: float = 0.12
    arm_pos_tolerance: float = 0.04
    arm_angle_tolerance: float = math.radians(7)
    max_target_speed_rate: float = 1.0
    max_torque_rate: float = 100.0
    max_steering_rate: float = math.radians(60)
    arm_poses: ArmMissionPoses = field(default_factory=ArmMissionPoses)

    def __post_init__(self):
        if self.arrival_threshold is None:
            self.arrival_threshold = self.zone_radius


@dataclass
class MissionCommand:
    torque: float = 0.0
    steering_angle: float = 0.0
    target_speed: float = 0.0


class NF1Path2DMotion:
    """Waypoint generator backed by the NF1 grid planner."""

    def __init__(self, scale=1.0, margin=5.0, waypoint_threshold=1.0, lookahead_steps=3):
        self.scale = scale
        self.margin = margin
        self.waypoint_threshold = waypoint_threshold
        self.lookahead_steps = lookahead_steps
        self.path = []
        self.current_index = 0
        self.offset = (0.0, 0.0)

    def start_motion(self, start, end):
        min_x = min(start[0], end[0])
        max_x = max(start[0], end[0])
        min_y = min(start[1], end[1])
        max_y = max(start[1], end[1])

        self.offset = (self.margin - min_x, self.margin - min_y)
        width = (max_x - min_x) + 2 * self.margin + self.scale
        height = (max_y - min_y) + 2 * self.margin + self.scale
        world = World(width, height, self.scale)

        planner = NF1Planner(world)
        plan_start = self._to_planner_point(start)
        plan_end = self._to_planner_point(end)
        grid_path = planner.plan(plan_start, plan_end)
        self.path = [self._to_mission_point(world.to_world(*p)) for p in grid_path]
        if self.path:
            self.path[0] = start
            self.path[-1] = end
        self.current_index = 1 if len(self.path) > 1 else 0

    def evaluate(self, _delta_t, current_pose=None):
        if not self.path:
            return (0.0, 0.0)

        if current_pose is not None:
            closest_index = min(
                range(self.current_index, len(self.path)),
                key=lambda i: math.hypot(
                    current_pose[0] - self.path[i][0],
                    current_pose[1] - self.path[i][1],
                ),
            )
            self.current_index = max(self.current_index, closest_index)
            while self.current_index < len(self.path) - 1:
                target = self.path[self.current_index]
                distance = math.hypot(current_pose[0] - target[0], current_pose[1] - target[1])
                if distance > self.waypoint_threshold:
                    break
                self.current_index += 1

        target_index = min(self.current_index + self.lookahead_steps, len(self.path) - 1)
        return self.path[target_index]

    def _to_planner_point(self, point):
        return point[0] + self.offset[0], point[1] + self.offset[1]

    def _to_mission_point(self, point):
        return point[0] - self.offset[0], point[1] - self.offset[1]


class CommandSmoother:
    def __init__(
        self,
        max_target_speed_rate: float,
        max_torque_rate: float,
        max_steering_rate: float,
    ):
        self.max_target_speed_rate = max_target_speed_rate
        self.max_torque_rate = max_torque_rate
        self.max_steering_rate = max_steering_rate
        self.command = MissionCommand()

    def evaluate_drive_target(
        self,
        delta_t: float,
        target_speed: float,
        steering_angle: float,
    ) -> tuple[float, float]:
        self.command.target_speed = self._limit_rate(
            self.command.target_speed,
            target_speed,
            self.max_target_speed_rate,
            delta_t,
        )
        self.command.steering_angle = self._limit_rate(
            self.command.steering_angle,
            steering_angle,
            self.max_steering_rate,
            delta_t,
        )
        return self.command.target_speed, self.command.steering_angle

    def evaluate_torque(self, delta_t: float, torque: float) -> float:
        self.command.torque = self._limit_rate(
            self.command.torque,
            torque,
            self.max_torque_rate,
            delta_t,
        )
        return self.command.torque

    def smooth_command(self, delta_t: float, target: MissionCommand) -> MissionCommand:
        self.command = MissionCommand(
            torque=self._limit_rate(
                self.command.torque,
                target.torque,
                self.max_torque_rate,
                delta_t,
            ),
            steering_angle=target.steering_angle,
            target_speed=target.target_speed,
        )
        return self.command

    def reset_drive_target(self):
        self.command.target_speed = 0.0
        self.command.steering_angle = 0.0

    @staticmethod
    def _limit_rate(current, target, max_rate, delta_t):
        if max_rate is None:
            return target
        max_delta = max_rate * delta_t
        delta = target - current
        if delta > max_delta:
            return current + max_delta
        if delta < -max_delta:
            return current - max_delta
        return target


class MissionState:
    state_id: MissionStateId
    cargo_phase: float

    def enter(self, mission):
        pass

    def update(
        self,
        mission,
        delta_t: float,
    ) -> tuple[MissionStateId, MissionCommand]:
        raise NotImplementedError


class DriveToPointState(MissionState):
    cargo_phase = 0.0

    def __init__(self, state_id, start_point, target_point, next_state):
        self.state_id = state_id
        self.start_point = start_point
        self.target_point = target_point
        self.next_state = next_state

    def enter(self, mission):
        mission.trajectory.start_motion(self.start_point, self.target_point)

    def update(self, mission, delta_t):
        pose = mission.cart.get_pose()
        current_speed, _ = mission.cart.get_speed()
        target_x, target_y = mission.trajectory.evaluate(delta_t, pose)
        target_speed, steering_angle = mission.position_controller.evaluate(
            delta_t, target_x, target_y, pose
        )
        target_speed, steering_angle = mission.command_smoother.evaluate_drive_target(
            delta_t, target_speed, steering_angle
        )
        torque = mission.speed_controller.evaluate(delta_t, target_speed - current_speed)

        distance = math.hypot(
            pose[0] - self.target_point[0],
            pose[1] - self.target_point[1],
        )
        if distance <= mission.config.arrival_threshold:
            if abs(current_speed) > mission.config.arrival_speed_threshold:
                return self.state_id, mission.hold_cart_command(delta_t)
            mission.log_arrival(self.target_point)
            return self.next_state, MissionCommand()

        return self.state_id, MissionCommand(torque, steering_angle, target_speed)


class DriveToAState(DriveToPointState):
    cargo_phase = 2.0


class ArmMoveState(MissionState):
    cargo_phase = 1.0

    def __init__(self, state_id, target_pose, next_state, message=None):
        self.state_id = state_id
        self.target_pose = target_pose
        self.next_state = next_state
        self.message = message

    def enter(self, mission):
        mission.set_arm_target_or_warn(self.target_pose, self.state_id.value)

    def update(self, mission, _delta_t):
        if mission.arm_is_near(self.target_pose):
            if self.message:
                mission.log(self.message)
            return self.next_state, mission.hold_cart_command(_delta_t)
        return self.state_id, mission.hold_cart_command(_delta_t)


class CargoArmState(ArmMoveState):
    def __init__(
        self,
        state_id,
        target_pose,
        next_state,
        zone,
        empty_message,
        message=None,
    ):
        super().__init__(state_id, target_pose, next_state, message)
        self.zone = zone
        self.empty_message = empty_message

    def update(self, mission, delta_t):
        next_state, command = super().update(mission, delta_t)
        if next_state == self.next_state:
            cargo_event = self.zone.process(mission.cart)
            if cargo_event is None:
                mission.log(self.empty_message)
            else:
                mission.cargo_events.append(cargo_event)
                mission.log("cargo_event: " + str(cargo_event))
        return next_state, command


class DropArmMoveState(ArmMoveState):
    cargo_phase = 3.0


class DropCargoArmState(CargoArmState):
    cargo_phase = 3.0


class DoneState(MissionState):
    state_id = MissionStateId.DONE
    cargo_phase = 4.0

    def update(self, _mission, _delta_t):
        return self.state_id, _mission.hold_cart_command(_delta_t)


class AckermannMissionController:
    """State-pattern controller for the shuttle cart, cargo zones and arm."""

    STATE_CODES = {
        MissionStateId.DRIVE_TO_B: 0,
        MissionStateId.PICK_APPROACH: 1,
        MissionStateId.PICK_DOWN: 2,
        MissionStateId.PICK_LIFT_SAFE: 3,
        MissionStateId.DRIVE_TO_A: 4,
        MissionStateId.DROP_APPROACH: 5,
        MissionStateId.DROP_DOWN: 6,
        MissionStateId.DROP_LIFT_SAFE: 7,
        MissionStateId.DONE: 8,
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
        self.trajectory = trajectory or NF1Path2DMotion()
        self.command_smoother = CommandSmoother(
            self.config.max_target_speed_rate,
            self.config.max_torque_rate,
            self.config.max_steering_rate,
        )
        self.logger = logger

        self.load_zone = PackageTransferZone(
            "B_shelf_load",
            self.config.point_b,
            self.config.zone_radius,
            PackageTransferZone.LOAD,
            self.config.packages_to_load,
        )
        self.unload_zone = PackageTransferZone(
            "A_unload",
            self.config.point_a,
            self.config.zone_radius,
            PackageTransferZone.UNLOAD,
        )
        self.cargo_events = []

        poses = self.config.arm_poses
        self.states = {
            MissionStateId.DRIVE_TO_B: DriveToPointState(
                MissionStateId.DRIVE_TO_B,
                self.config.point_a,
                self.config.point_b,
                MissionStateId.PICK_APPROACH,
            ),
            MissionStateId.PICK_APPROACH: ArmMoveState(
                MissionStateId.PICK_APPROACH,
                poses.pick_approach,
                MissionStateId.PICK_DOWN,
                "Braccio sopra il pacco: scendo per prendere",
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
                MissionStateId.DRIVE_TO_A,
                "Braccio sicuro: parto verso il punto A per scaricare",
            ),
            MissionStateId.DRIVE_TO_A: DriveToAState(
                MissionStateId.DRIVE_TO_A,
                self.config.point_b,
                self.config.point_a,
                MissionStateId.DROP_APPROACH,
            ),
            MissionStateId.DROP_APPROACH: DropArmMoveState(
                MissionStateId.DROP_APPROACH,
                poses.drop_approach,
                MissionStateId.DROP_DOWN,
                "Braccio sopra la zona di scarico: scendo",
            ),
            MissionStateId.DROP_DOWN: DropCargoArmState(
                MissionStateId.DROP_DOWN,
                poses.drop_down,
                MissionStateId.DROP_LIFT_SAFE,
                self.unload_zone,
                "Nessun cargo_event in unload: controlla posizione del carrello o zone_radius",
                "Pacco scaricato: riporto il braccio in posizione sicura",
            ),
            MissionStateId.DROP_LIFT_SAFE: DropArmMoveState(
                MissionStateId.DROP_LIFT_SAFE,
                poses.safe,
                MissionStateId.DONE,
                "Missione completata",
            ),
            MissionStateId.DONE: DoneState(),
        }

        self.state_id = MissionStateId.DRIVE_TO_B
        self.command = MissionCommand()
        self.cart.set_pose(self.config.point_a[0], self.config.point_a[1])
        self.set_arm_target_or_warn(poses.safe, "ARM_SAFE_POSE")
        self.states[self.state_id].enter(self)

    def log(self, message):
        if self.logger is not None:
            self.logger(message)

    def log_arrival(self, point):
        if point == self.config.point_b:
            self.log("Arrivato al punto B: inizio presa dallo scaffale")
        elif point == self.config.point_a:
            self.log("Arrivato al punto A: preparo lo scarico")

    def set_arm_target_or_warn(self, target_pose, label):
        ok = self.arm.set_target(*target_pose)
        if not ok:
            self.log("WARNING: target braccio non raggiungibile: " + str((label, target_pose)))
        return ok

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

    def cargo_phase(self):
        return self.states[self.state_id].cargo_phase

    def final_report(self):
        final_pose = self.cart.get_pose_3d()
        final_error_a = math.hypot(
            final_pose[0] - self.config.point_a[0],
            final_pose[1] - self.config.point_a[1],
        )
        reached_a = final_error_a <= self.config.arrival_threshold
        mission_complete = (
            self.is_done()
            and reached_a
            and self.cart.get_payload_mass() == 0
        )
        return {
            "point_a": self.config.point_a,
            "point_b": self.config.point_b,
            "zone_radius_m": self.config.zone_radius,
            "cargo_events": self.cargo_events,
            "final_pose": final_pose,
            "final_error_from_A_m": final_error_a,
            "payload_kg": self.cart.get_payload_mass(),
            "reached_A": reached_a,
            "mission_complete": mission_complete,
        }
