from dataclasses import dataclass, field
import math


@dataclass
class ArmMissionPoses:
    safe: tuple = (0.20, 0.00, 0.55, math.radians(-90))
    pick_approach: tuple = (0.25, 0.45, 0.40, math.radians(-90))
    pick_down: tuple = (0.25, 0.45, 0.12, math.radians(-90))
    drop_approach: tuple = (0.25, 0.00, 0.35, math.radians(-90))
    drop_down: tuple = (0.25, 0.00, 0.08, math.radians(-90))


@dataclass
class VisionTargetConfig:
    colors: tuple = ("blue", "red")
    min_area: int = 80
    max_area: int = 250000


@dataclass
class CargoRoutingConfig:
    unload_zone_by_color: dict = field(default_factory=lambda: {
        "blue": "A",
        "red": "C",
    })
    default_unload_zone: str = "A"
    color_codes: dict = field(default_factory=lambda: {
        "blue": 1,
        "red": 2,
    })
    unload_zone_codes: dict = field(default_factory=lambda: {
        "A": 1,
        "C": 3,
    })


@dataclass
class AckermannMissionConfig:
    point_a: tuple = (0.0, 0.0)
    point_b: tuple = (0.0, -25.0)#0 -25
    point_c: tuple = (18, -41)
    navigation_obstacles: tuple = field(default_factory=lambda: (
        ((12, -34.0), (1.0, 6.0)), ##prima 0,20 ha fatto 0,-20
    ))
    zone_radius: float = 0.35
    packages_to_load: list = field(default_factory=lambda: [5.0, 5.0, 5.0])
    cargo_routing: CargoRoutingConfig = field(default_factory=CargoRoutingConfig)
    vision_targets: VisionTargetConfig | None = None
    vision_target_colors: tuple | None = None
    vision_target_min_area: int | None = None
    vision_target_max_area: int | None = None
    arrival_threshold: float | None = None
    arrival_speed_threshold: float = 0.12
    arm_pos_tolerance: float = 0.04
    arm_angle_tolerance: float = math.radians(7)
    max_target_speed_rate: float = 1.0
    max_torque_rate: float = 100.0
    max_steering_rate: float = math.radians(60)
    arm_poses: ArmMissionPoses = field(default_factory=ArmMissionPoses)
    vision_pick_enabled: bool = True
    vision_center: tuple = (256, 256)
    vision_pixel_tolerance: int = 8
    vision_lock_frames: int = 4
    vision_sample_period: float = 0.12
    vision_pixel_to_arm_gain: float = 0.0005
    vision_max_adjust_step: float = 0.04
    vision_scan_dwell: float = 1.5
    vision_scan_poses: tuple | None = None
    vision_scan_radius: float = 0.80
    vision_scan_heights: tuple = (0.35, 0.50, 0.65)
    vision_scan_steps: int = 16
    vision_scan_start_angle: float = math.radians(-180)
    vision_min_track_area: int = 2500
    vision_track_margin_px: int = 20
    vision_cart_approach_enabled: bool = True
    vision_cart_approach_speed: float = 0.22
    vision_cart_approach_step: float = 0.35
    vision_cart_max_approach: float = 2.0
    vision_track_pose_timeout: float = 0.5

    def __post_init__(self):
        if self.vision_targets is None:
            self.vision_targets = VisionTargetConfig(
                colors=self.vision_target_colors or tuple(
                    self.cargo_routing.unload_zone_by_color.keys()
                ),
                min_area=(
                    80 if self.vision_target_min_area is None
                    else self.vision_target_min_area
                ),
                max_area=(
                    250000 if self.vision_target_max_area is None
                    else self.vision_target_max_area
                ),
            )
        self.vision_target_colors = self.vision_targets.colors
        self.vision_target_min_area = self.vision_targets.min_area
        self.vision_target_max_area = self.vision_targets.max_area
        if self.arrival_threshold is None:
            self.arrival_threshold = 0.65
        if self.vision_scan_poses is None:
            self.vision_scan_poses = self._build_vision_scan_poses()

    @property
    def unload_zone_by_color(self):
        return self.cargo_routing.unload_zone_by_color

    @property
    def default_unload_zone(self):
        return self.cargo_routing.default_unload_zone

    def cargo_color_code(self, color):
        if color is None:
            return 0
        return self.cargo_routing.color_codes.get(color, 0)

    def unload_zone_code(self, label):
        return self.cargo_routing.unload_zone_codes.get(label, 0)

    def unload_points(self):
        return {
            "A": self.point_a,
            "C": self.point_c,
        }

    def _build_vision_scan_poses(self):
        poses = []
        steps = max(1, self.vision_scan_steps)
        for height in self.vision_scan_heights:
            for index in range(steps):
                angle = self.vision_scan_start_angle + (2.0 * math.pi * index / steps)
                poses.append((
                    self.vision_scan_radius * math.cos(angle),
                    self.vision_scan_radius * math.sin(angle),
                    height,
                    math.radians(-90),
                ))
        return tuple(poses)
