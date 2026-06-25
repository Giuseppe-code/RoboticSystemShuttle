try:
    from .base import MissionState
except ImportError:
    from states.base import MissionState


class ArmMoveState(MissionState):
    def __init__(self, state_id, target_pose, next_state, message=None):
        self.state_id = state_id
        self.target_pose = target_pose
        self.next_state = next_state
        self.message = message

    def target_for(self, mission):
        return self.target_pose

    def enter(self, mission):
        mission.set_arm_target_or_warn(self.target_for(mission), self.state_id.value)

    def update(self, mission, _delta_t):
        target_pose = self.target_for(mission)
        if mission.arm_is_near(target_pose):
            if self.message:
                mission.log(self.message)
            return self.next_state, mission.hold_cart_command(_delta_t)
        return self.state_id, mission.hold_cart_command(_delta_t)
