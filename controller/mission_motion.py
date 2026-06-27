import math
import cv2

try:
    from .mission_types import MissionCommand
except ImportError:
    from mission_types import MissionCommand

try:
    from .nf1 import NF1Planner
    from .world import World
except ImportError:
    from nf1 import NF1Planner
    from world import World


class NF1Path2DMotion:
    """Waypoint generator backed by the NF1 grid planner."""

    def __init__(
        self,
        scale=1.0,
        margin=5.0,
        waypoint_threshold=1.0,
        lookahead_steps=3,
        obstacles=None,
        obstacle_padding=2.0,
    ):
        self.numImg=0
        self.scale = scale
        self.margin = margin
        self.waypoint_threshold = waypoint_threshold
        self.lookahead_steps = lookahead_steps
        self.obstacles = tuple(obstacles or ())
        self.obstacle_padding = obstacle_padding
        self.path = []
        self.current_index = 0
        self.offset = (0.0, 0.0)
        self.planned_paths = {}

    def start_motion(self, start, end):
        self.path = list(self.preplan_motion(start, end))
        self.current_index = 1 if len(self.path) > 1 else 0

    def preplan_motion(self, start, end):
        route_key = self._route_key(start, end)
        if route_key not in self.planned_paths:
            self.planned_paths[route_key] = self._build_path(start, end)
        return self.planned_paths[route_key]

    def _build_path(self, start, end):
        min_x = min(start[0], end[0])
        max_x = max(start[0], end[0])
        min_y = min(start[1], end[1])
        max_y = max(start[1], end[1])

        margin = self.margin + self.obstacle_padding
        self.offset = (margin - min_x, margin - min_y)
        width = (max_x - min_x) + 2 * margin + self.scale
        height = (max_y - min_y) + 2 * margin + self.scale
        world = World(width, height, self.scale)
        for obstacle in self.obstacles:
            world.add_rectangle_obstacle(*self._obstacle_bounds(obstacle))

        planner = NF1Planner(world)
        plan_start = self._to_planner_point(start)
        plan_end = self._to_planner_point(end)
        grid_path = planner.plan(plan_start, plan_end)
        img2 = planner.world_to_image()
        cv2.imwrite(f"debug_nf{self.numImg}_path.png", img2)  # salva su file
        self.numImg=self.numImg+1
        path = [self._to_mission_point(world.to_world(*p)) for p in grid_path]
        if path:
            path[0] = start
            path[-1] = end
        return path

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

    def _obstacle_bounds(self, obstacle):
        center, size = obstacle
        half_width = size[0] / 2.0 + self.obstacle_padding
        half_height = size[1] / 2.0 + self.obstacle_padding
        min_point = self._to_planner_point((center[0] - half_width, center[1] - half_height))
        max_point = self._to_planner_point((center[0] + half_width, center[1] + half_height))
        return min_point[0], min_point[1], max_point[0], max_point[1]

    def _route_key(self, start, end):
        return tuple(start), tuple(end)


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
