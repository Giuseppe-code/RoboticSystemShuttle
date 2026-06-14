import math
import unittest

from controller.ackermann_mission import (
    AckermannMissionConfig,
    AckermannMissionController,
    MissionStateId,
)
from controller.manipulator_control import FourJointsManipulatorControl
from lib.system.cart import AckermannSlopeLoad, TerrainProfile


class InstantArm:
    def __init__(self):
        self.pose = (0.0, 0.0, 0.0, 0.0)

    def set_target(self, x, y, z, a):
        self.pose = (x, y, z, a)
        return True

    def get_pose(self):
        return self.pose

    def evaluate(self, _delta_t):
        pass


class AckermannMissionControllerTest(unittest.TestCase):
    def test_first_drive_command_is_smoothed(self):
        config = AckermannMissionConfig(
            point_a=(0.0, 0.0),
            point_b=(0.0, -25.0),
            max_target_speed_rate=1.0,
            max_steering_rate=math.radians(60),
        )
        cart = AckermannSlopeLoad(
            10.0,
            0.8,
            0.5,
            2.0,
            TerrainProfile([(0.0, 100.0, 0.0)]),
        )
        mission = AckermannMissionController(
            cart,
            InstantArm(),
            config=config,
            logger=None,
        )

        delta_t = 0.01
        command = mission.update(delta_t)

        self.assertLessEqual(
            abs(command.target_speed),
            config.max_target_speed_rate * delta_t,
        )
        self.assertLessEqual(
            abs(command.steering_angle),
            config.max_steering_rate * delta_t,
        )

    def test_state_machine_loads_at_b_and_unloads_at_a(self):
        config = AckermannMissionConfig(
            point_a=(0.0, 0.0),
            point_b=(0.0, -1.0),
            zone_radius=0.25,
            packages_to_load=[5.0, 5.0, 5.0],
        )
        cart = AckermannSlopeLoad(
            10.0,
            0.8,
            0.5,
            2.0,
            TerrainProfile([(0.0, 100.0, 0.0)]),
        )
        mission = AckermannMissionController(
            cart,
            InstantArm(),
            config=config,
            logger=None,
        )

        cart.set_pose(*config.point_b)
        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.PICK_APPROACH)

        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.PICK_DOWN)

        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.PICK_LIFT_SAFE)
        self.assertEqual(cart.get_payload_mass(), 5.0)

        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.DRIVE_TO_A)

        cart.set_pose(*config.point_a)
        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.DROP_APPROACH)

        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.DROP_DOWN)

        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.DROP_LIFT_SAFE)
        self.assertEqual(cart.get_payload_mass(), 0.0)

        mission.update(0.1)
        self.assertTrue(mission.is_done())
        self.assertEqual(len(mission.cargo_events), 2)

    def test_full_mission_completes_with_real_arm_and_vehicle_dynamics(self):
        config = AckermannMissionConfig(
            point_a=(0.0, 0.0),
            point_b=(0.0, -25.0),
            zone_radius=0.35,
            packages_to_load=[5.0, 5.0, 5.0],
        )
        cart = AckermannSlopeLoad(
            10.0,
            0.8,
            0.5,
            2.0,
            TerrainProfile([(0.0, 200.0, 0.0)]),
        )
        mission = AckermannMissionController(
            cart,
            FourJointsManipulatorControl(),
            config=config,
            logger=None,
        )

        delta_t = 0.01
        for _ in range(int(80 / delta_t)):
            mission.step(delta_t)
            if mission.is_done():
                break

        report = mission.final_report()
        self.assertTrue(report["mission_complete"])
        self.assertEqual(report["payload_kg"], 0)
        self.assertEqual(len(report["cargo_events"]), 2)


if __name__ == "__main__":
    unittest.main()
