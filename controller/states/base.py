from mission_types import MissionCommand, MissionStateId


class MissionState:
    state_id: MissionStateId

    def enter(self, mission):
        pass

    def update(
        self,
        mission,
        delta_t: float,
    ) -> tuple[MissionStateId, MissionCommand]:
        raise NotImplementedError
