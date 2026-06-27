from .cargo_arm import CargoArmState


class DropCargoArmState(CargoArmState):
    def zone_for(self, mission):
        return mission.selected_unload_zone()
