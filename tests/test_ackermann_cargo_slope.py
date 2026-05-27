import math
import unittest

from lib.system.cart import (
    AckermannSlopeLoad,
    MeasuredTerrainProfile,
    PackageTransferZone,
    TerrainProfile,
    package_is_stable,
)


class AckermannSlopeLoadTest(unittest.TestCase):
    def test_weight_and_slope_are_reflected_in_force_and_pose(self):
        terrain = TerrainProfile([(0.0, 100.0, 10.0)])
        cart = AckermannSlopeLoad(10.0, 0.8, 0.5, 2.0, terrain, [5.0, 5.0, 5.0])

        cart.evaluate(0.1, torque=100.0, steering_angle=0.0)
        pose = cart.get_pose_3d()

        self.assertEqual(cart.get_total_mass(), 25.0)
        self.assertAlmostEqual(cart.slope_force, 25.0 * 9.81 * math.sin(math.radians(10.0)))
        self.assertGreater(pose[2], 0.0)
        self.assertAlmostEqual(pose[4], math.radians(10.0))

    def test_measured_map_sample_drives_height_and_slope(self):
        terrain = MeasuredTerrainProfile()
        terrain.update(distance=4.0, height=1.25, slope=math.radians(8.0))

        self.assertAlmostEqual(terrain.slope_at(5.0), math.radians(8.0))
        self.assertAlmostEqual(terrain.height_at(6.0), 1.25 + 2.0 * math.sin(math.radians(8.0)))

    def test_package_stability_detects_insufficient_friction(self):
        terrain = TerrainProfile([(0.0, 100.0, 30.0)])
        cart = AckermannSlopeLoad(10.0, 0.0, 0.5, 2.0, terrain)
        holding_torque = 10.0 * 9.81 * math.sin(math.radians(30.0)) * 0.5
        cart.evaluate(0.1, torque=holding_torque, steering_angle=0.0)

        self.assertFalse(package_is_stable(cart, package_mass=5.0, mu_static=0.1))


class PackageTransferZoneTest(unittest.TestCase):
    def test_loads_in_a_and_unloads_in_b_only_inside_confidence_radius(self):
        cart = AckermannSlopeLoad(
            10.0, 0.8, 0.5, 2.0, TerrainProfile([(0.0, 100.0, 0.0)])
        )
        load_zone = PackageTransferZone("A", (0.0, 0.0), 0.25, "load", [5.0, 5.0, 5.0])
        unload_zone = PackageTransferZone("B", (5.0, 5.0), 0.25, "unload")

        load_event = load_zone.process(cart)
        self.assertEqual(load_event["payload_after_kg"], 15.0)
        self.assertEqual(cart.get_total_mass(), 25.0)
        self.assertIsNone(load_zone.process(cart))
        self.assertIsNone(unload_zone.process(cart))

        cart.set_pose(4.9, 5.0)
        unload_event = unload_zone.process(cart)
        self.assertEqual(unload_event["payload_after_kg"], 0)
        self.assertEqual(cart.get_total_mass(), 10.0)


if __name__ == "__main__":
    unittest.main()
