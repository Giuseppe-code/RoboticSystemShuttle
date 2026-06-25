import argparse
import json
import math
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "controller"))

from lib.dds.dds import DDS
from lib.dds.image_reader import ImageReader
from lib.system.cart import AckermannSlopeLoad, MeasuredTerrainProfile
from lib.system.object_finder import ObjectFinder
from lib.utils.time import Time
from manipulator_control import FourJointsManipulatorControl
from ackermann_mission import AckermannMissionConfig, AckermannMissionController


class VisionProvider:
    def __init__(self, dds, mission_config, host="localhost", port=4445, output_dir=None):
        self.dds = dds
        self.reader = ImageReader(host, port)
        self.finders = {
            color: ObjectFinder(
                color,
                min_area=mission_config.vision_targets.min_area,
                max_area=mission_config.vision_targets.max_area,
            )
            for color in mission_config.vision_targets.colors
        }
        self.output_dir = output_dir
        self.sample_count = 0
        self.detection_count = 0
        self.last_detection = None

    def connect(self):
        self.reader.connect()

    def __call__(self):
        image = self.reader.request_image(self.dds, 512, 512)
        self.sample_count += 1

        best = None
        best_image = image.copy()
        best_mask = None
        for color, finder in self.finders.items():
            candidate = image.copy()
            cx, cy, mask = finder.find(candidate)
            if cx < 0 or cy < 0:
                continue
            area = self._largest_area(mask)
            detection = {
                "color": color,
                "cx": cx,
                "cy": cy,
                "area": area,
            }
            if best is None or area > best["area"]:
                best = detection
                best_image = candidate
                best_mask = mask

        if best is not None:
            self.detection_count += 1
            self.last_detection = best
            print(
                "[vision] color={color} cx={cx} cy={cy} area={area:.0f}".format(
                    **best
                ),
                flush=True,
            )
            self._save_debug_frame(best_image, best_mask)
        else:
            print("[vision] no target", flush=True)
        return best

    @staticmethod
    def _largest_area(mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if not contours:
            return 0.0
        return float(max(cv2.contourArea(contour) for contour in contours))

    def _save_debug_frame(self, image, mask):
        if self.output_dir is None:
            return
        if self.detection_count not in (1, 5, 10, 20):
            return
        self.output_dir.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.output_dir / f"vision_{self.detection_count:02d}.png"), image)
        if mask is not None:
            cv2.imwrite(str(self.output_dir / f"vision_{self.detection_count:02d}_mask.png"), mask)


def publish_state(dds, cart, arm, mission, command):
    pose_3d = cart.get_pose_3d()
    theta0, theta1, theta2, theta3 = arm.get_joint_angles()
    arm_x, arm_y, arm_z, arm_a = arm.get_pose()

    dds.publish("X", pose_3d[0], DDS.DDS_TYPE_FLOAT)
    dds.publish("Y", pose_3d[1], DDS.DDS_TYPE_FLOAT)
    dds.publish("Z", pose_3d[2], DDS.DDS_TYPE_FLOAT)
    dds.publish("Theta", pose_3d[3], DDS.DDS_TYPE_FLOAT)
    dds.publish("Slope", pose_3d[4], DDS.DDS_TYPE_FLOAT)
    dds.publish("PayloadMass", cart.get_payload_mass(), DDS.DDS_TYPE_FLOAT)
    dds.publish("MissionState", float(mission.state_code()), DDS.DDS_TYPE_FLOAT)
    dds.publish("CargoColorCode", float(mission.selected_cargo_color_code()), DDS.DDS_TYPE_FLOAT)
    dds.publish("UnloadZoneCode", float(mission.selected_unload_zone_code()), DDS.DDS_TYPE_FLOAT)

    dds.publish("theta0", theta0, DDS.DDS_TYPE_FLOAT)
    dds.publish("theta1", theta1, DDS.DDS_TYPE_FLOAT)
    dds.publish("theta2", theta2, DDS.DDS_TYPE_FLOAT)
    dds.publish("theta3", theta3, DDS.DDS_TYPE_FLOAT)
    dds.publish("arm_x", arm_x, DDS.DDS_TYPE_FLOAT)
    dds.publish("arm_y", arm_y, DDS.DDS_TYPE_FLOAT)
    dds.publish("arm_z", arm_z, DDS.DDS_TYPE_FLOAT)
    dds.publish("arm_a", arm_a, DDS.DDS_TYPE_FLOAT)

    dds.publish("x", arm_x, DDS.DDS_TYPE_FLOAT)
    dds.publish("y", arm_y, DDS.DDS_TYPE_FLOAT)
    dds.publish("z", arm_z, DDS.DDS_TYPE_FLOAT)
    dds.publish("a", arm_a, DDS.DDS_TYPE_FLOAT)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument("--point-b-y", type=float, default=-25.0)
    parser.add_argument("--point-c-x", type=float, default=5.0)
    parser.add_argument("--point-c-y", type=float, default=0.0)
    parser.add_argument("--zone-radius", type=float, default=0.35)
    parser.add_argument("--vision-tolerance", type=int, default=70)
    parser.add_argument("--vision-lock-frames", type=int, default=1)
    parser.add_argument("--godot-delta", action="store_true")
    parser.add_argument("--no-vision", action="store_true")
    parser.add_argument("--debug-dir", type=Path, default=ROOT / "logs" / "vision")
    args = parser.parse_args()

    dds = DDS()
    dds.start()
    dds.subscribe(["tick", "TerrainHeight", "TerrainSlope"])

    vision = None
    try:
        mission_config = AckermannMissionConfig(
            point_a=(0.0, 0.0),
            point_b=(0.0, args.point_b_y),
            point_c=(args.point_c_x, args.point_c_y),
            zone_radius=args.zone_radius,
            packages_to_load=[5.0, 5.0, 5.0],
            vision_pixel_tolerance=args.vision_tolerance,
            vision_lock_frames=args.vision_lock_frames,
        )
        if not args.no_vision:
            vision = VisionProvider(dds, mission_config, output_dir=args.debug_dir)
            print("[startup] connecting camera stream on localhost:4445", flush=True)
            vision.connect()
            print("[startup] camera stream connected", flush=True)

        arm = FourJointsManipulatorControl()
        terrain = MeasuredTerrainProfile()
        cart = AckermannSlopeLoad(
            cart_mass=10,
            friction=0.8,
            wheel_radius=0.5,
            wheelbase=2.0,
            terrain=terrain,
        )
        mission = AckermannMissionController(
            cart,
            arm,
            mission_config,
            vision_callback=vision,
        )

        timer = Time()
        timer.start()
        sim_time = 0.0
        last_status = -1.0
        while sim_time < args.duration:
            tick_delta = dds.wait("tick")
            delta_t = float(tick_delta) if args.godot_delta else timer.elapsed()
            sim_time += delta_t

            terrain_height = dds.read("TerrainHeight")
            terrain_slope = dds.read("TerrainSlope")
            if terrain_height is not None and terrain_slope is not None:
                terrain.update(cart.s, terrain_height, terrain_slope)

            command = mission.step(delta_t)
            publish_state(dds, cart, arm, mission, command)

            if sim_time - last_status >= 2.0:
                pose = cart.get_pose_3d()
                print(
                    "[status] t={:.1f}s state={} pose=({:.2f},{:.2f},{:.2f}) payload={:.1f}kg".format(
                        sim_time,
                        mission.state_name(),
                        pose[0],
                        pose[1],
                        pose[2],
                        cart.get_payload_mass(),
                    ),
                    flush=True,
                )
                last_status = sim_time

            if mission.is_done():
                break

        report = mission.final_report()
        report["vision_samples"] = vision.sample_count if vision else 0
        report["vision_detections"] = vision.detection_count if vision else 0
        report["last_vision_detection"] = vision.last_detection if vision else None
        print("[report] " + json.dumps(report, indent=2, default=str), flush=True)
        return 0 if report["mission_complete"] else 2
    finally:
        dds.stop()


if __name__ == "__main__":
    raise SystemExit(main())
