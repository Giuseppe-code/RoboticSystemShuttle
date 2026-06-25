from dataclasses import dataclass
from enum import Enum


class MissionStateId(Enum):
    DRIVE_TO_B = "DRIVE_TO_B"
    PICK_APPROACH = "PICK_APPROACH"
    VISION_PICK = "VISION_PICK"
    PICK_DOWN = "PICK_DOWN"
    PICK_LIFT_SAFE = "PICK_LIFT_SAFE"
    DRIVE_TO_UNLOAD = "DRIVE_TO_UNLOAD"
    DROP_APPROACH = "DROP_APPROACH"
    DROP_DOWN = "DROP_DOWN"
    DROP_LIFT_SAFE = "DROP_LIFT_SAFE"
    DONE = "DONE"


@dataclass
class MissionCommand:
    torque: float = 0.0
    steering_angle: float = 0.0
    target_speed: float = 0.0
