import math
from mission_types import MissionCommand
from states.base import MissionState


class DriveToPointState(MissionState):
    def __init__(self, state_id, start_point, target_point, next_state):
        self.state_id = state_id
        self.start_point = start_point
        self.target_point = target_point
        self.next_state = next_state
        self.active_target_point = target_point

    def enter(self, mission):
        self.active_target_point = self.target_for(mission)
        mission.trajectory.start_motion(self.start_for(mission), self.active_target_point)

    def start_for(self, _mission):
        return self.start_point

    def target_for(self, _mission):
        return self.target_point

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
            pose[0] - self.active_target_point[0],
            pose[1] - self.active_target_point[1],
        )
        if distance <= mission.config.arrival_threshold:
            if abs(current_speed) > mission.config.arrival_speed_threshold:
                return self.state_id, mission.hold_cart_command(delta_t)
            mission.log_arrival(self.active_target_point)
            return self.next_state, MissionCommand()

        return self.state_id, MissionCommand(torque, steering_angle, target_speed)
