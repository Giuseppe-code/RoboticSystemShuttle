from .drive_to_point import DriveToPointState


class DriveToUnloadState(DriveToPointState):
    def target_for(self, mission):
        return mission.selected_unload_point()
