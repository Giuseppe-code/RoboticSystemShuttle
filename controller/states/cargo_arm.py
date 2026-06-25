try:
    from ..mission_types import MissionStateId
    from .arm_move import ArmMoveState
except ImportError:
    from mission_types import MissionStateId
    from states.arm_move import ArmMoveState


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

    def target_for(self, mission):
        if (
            self.state_id == MissionStateId.PICK_DOWN
            and mission.vision_pick_down_pose is not None
        ):
            return mission.vision_pick_down_pose
        return self.target_pose

    def zone_for(self, _mission):
        return self.zone

    def update(self, mission, delta_t):
        next_state, command = super().update(mission, delta_t)
        if next_state == self.next_state:
            cargo_event = self.zone_for(mission).process(mission.cart)
            if cargo_event is None:
                mission.log(self.empty_message)
            else:
                mission.cargo_events.append(cargo_event)
                mission.log("cargo_event: " + str(cargo_event))
        return next_state, command
