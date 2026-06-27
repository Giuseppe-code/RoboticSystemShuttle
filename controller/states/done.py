from mission_types import MissionStateId
from states.base import MissionState


class DoneState(MissionState):
    state_id = MissionStateId.DONE

    def update(self, _mission, _delta_t):
        return self.state_id, _mission.hold_cart_command(_delta_t)
