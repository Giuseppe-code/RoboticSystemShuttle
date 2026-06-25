import math
import unittest

from controller.ackermann_mission import (
    AckermannMissionConfig,
    AckermannMissionController,
    NF1Path2DMotion,
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
    def test_default_vision_scan_covers_full_rotation(self):
        config = AckermannMissionConfig(
            vision_scan_radius=0.8,
            vision_scan_heights=(0.35, 0.5),
            vision_scan_steps=8,
        )

        self.assertEqual(len(config.vision_scan_poses), 16)
        first_ring = config.vision_scan_poses[:8]
        angles = [math.atan2(y, x) for x, y, _z, _a in first_ring]
        self.assertAlmostEqual(angles[0], -math.pi)
        self.assertAlmostEqual(angles[-1], math.radians(135))
        for x, y, _z, _a in config.vision_scan_poses:
            self.assertAlmostEqual(math.hypot(x, y), 0.8)

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

    def test_state_machine_loads_at_b_and_unloads_at_default_a(self):
        config = AckermannMissionConfig(
            point_a=(0.0, 0.0),
            point_b=(0.0, -1.0),
            point_c=(1.0, 0.0),
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
        self.assertEqual(mission.state_id, MissionStateId.VISION_PICK)

        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.PICK_DOWN)

        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.PICK_LIFT_SAFE)
        self.assertEqual(cart.get_payload_mass(), 5.0)

        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.DRIVE_TO_UNLOAD)
        self.assertEqual(mission.selected_unload_label(), "A")

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
        self.assertEqual(mission.cargo_events[-1]["zone"], "A_unload")

    def test_red_package_unloads_at_c(self):
        config = AckermannMissionConfig(
            point_a=(0.0, 0.0),
            point_b=(0.0, -1.0),
            point_c=(1.0, 0.0),
            zone_radius=0.25,
            packages_to_load=[5.0],
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
        mission.update(0.1)
        mission.update(0.1)
        mission.update(0.1)
        mission.selected_cargo_color = "red"

        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.DRIVE_TO_UNLOAD)
        self.assertEqual(mission.selected_unload_label(), "C")

        cart.set_pose(*config.point_c)
        mission.update(0.1)
        mission.update(0.1)
        mission.update(0.1)

        self.assertEqual(cart.get_payload_mass(), 0.0)
        self.assertEqual(mission.cargo_events[-1]["zone"], "C_unload")

    def test_nf1_preplans_routes_between_b_and_c(self):
        config = AckermannMissionConfig(
            point_a=(0.0, 0.0),
            point_b=(0.0, -1.0),
            point_c=(1.0, 0.0),
            zone_radius=0.25,
        )
        cart = AckermannSlopeLoad(
            10.0,
            0.8,
            0.5,
            2.0,
            TerrainProfile([(0.0, 100.0, 0.0)]),
        )
        trajectory = NF1Path2DMotion()
        AckermannMissionController(
            cart,
            InstantArm(),
            config=config,
            trajectory=trajectory,
            logger=None,
        )

        self.assertIn((config.point_b, config.point_c), trajectory.planned_paths)
        self.assertIn((config.point_c, config.point_b), trajectory.planned_paths)

    def test_vision_pick_creeps_forward_after_empty_scan(self):
        config = AckermannMissionConfig(
            point_a=(0.0, 0.0),
            point_b=(0.0, -1.0),
            zone_radius=0.25,
            vision_scan_poses=((0.2, 0.0, 0.4, math.radians(-90)),),
            vision_scan_dwell=0.01,
            vision_sample_period=0.01,
            vision_cart_approach_speed=0.2,
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
            vision_callback=lambda: None,
        )

        cart.set_pose(*config.point_b)
        mission.update(0.1)
        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.VISION_PICK)

        mission.update(0.1)
        command = mission.update(0.1)
        self.assertGreater(command.target_speed, 0.0)

    def test_vision_pick_creeps_forward_when_target_is_on_image_edge(self):
        config = AckermannMissionConfig(
            point_a=(0.0, 0.0),
            point_b=(0.0, -1.0),
            zone_radius=0.25,
            vision_scan_poses=((0.2, 0.0, 0.4, math.radians(-90)),),
            vision_scan_dwell=0.01,
            vision_sample_period=0.01,
            vision_min_track_area=2500,
            vision_track_margin_px=40,
            vision_cart_approach_speed=0.2,
        )
        cart = AckermannSlopeLoad(
            10.0,
            0.8,
            0.5,
            2.0,
            TerrainProfile([(0.0, 100.0, 0.0)]),
        )
        detection = {"color": "blue", "cx": 256, "cy": 12, "area": 7000}
        mission = AckermannMissionController(
            cart,
            InstantArm(),
            config=config,
            logger=None,
            vision_callback=lambda: detection,
        )

        cart.set_pose(*config.point_b)
        mission.update(0.1)
        mission.update(0.1)
        self.assertEqual(mission.state_id, MissionStateId.VISION_PICK)

        mission.update(0.1)
        command = mission.update(0.1)
        self.assertGreater(command.target_speed, 0.0)
        self.assertIsNone(mission.vision_pick_down_pose)

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
