import math
import unittest

from kinematics import SwerveParams, calculate_swerve, world_to_screen


class TestSwerveKinematics(unittest.TestCase):
    def test_world_to_screen_center_and_axes(self):
        # Center (0, 0) maps to (cx, cy); +X goes right, +Y goes up (smaller screen y)
        sx, sy = world_to_screen(0.0, 0.0, cx=200.0, cy=150.0, scale=100.0)
        self.assertEqual((sx, sy), (200.0, 150.0))

        sx_pos, sy_pos = world_to_screen(1.0, 0.5, cx=200.0, cy=150.0, scale=100.0)
        self.assertAlmostEqual(sx_pos, 300.0, places=6)
        self.assertAlmostEqual(sy_pos, 100.0, places=6)

    def test_pure_rotation_matches_reference(self):
        params = SwerveParams(
            velocity=0.0,
            alpha_deg=0.0,
            yaw_deg=0.0,
            omega=2.0,
            wheelbase=1.2,
            track_width=1.2,
        )
        result = calculate_swerve(params)
        expected_speed = math.sqrt(1.2**2 + 1.2**2)

        self.assertAlmostEqual(result.wheels["FL"].speed, expected_speed, places=6)
        self.assertAlmostEqual(result.wheels["FR"].speed, expected_speed, places=6)
        self.assertAlmostEqual(result.wheels["BL"].speed, expected_speed, places=6)
        self.assertAlmostEqual(result.wheels["BR"].speed, expected_speed, places=6)

        self.assertAlmostEqual(result.wheels["FL"].steering_deg, 135.0, places=6)
        self.assertAlmostEqual(result.wheels["FR"].steering_deg, 45.0, places=6)
        self.assertAlmostEqual(result.wheels["BL"].steering_deg, -135.0, places=6)
        self.assertAlmostEqual(result.wheels["BR"].steering_deg, -45.0, places=6)

    def test_pure_translation_along_y(self):
        params = SwerveParams(
            velocity=5.0,
            alpha_deg=90.0,
            yaw_deg=0.0,
            omega=0.0,
            wheelbase=1.2,
            track_width=1.2,
        )
        result = calculate_swerve(params)
        for wheel in result.wheels.values():
            self.assertAlmostEqual(wheel.speed, 5.0, places=6)
            self.assertAlmostEqual(wheel.steering_deg, 90.0, places=6)

    def test_yaw_zero_points_front_to_positive_y(self):
        params = SwerveParams(
            velocity=5.0,
            alpha_deg=0.0,
            yaw_deg=0.0,
            omega=0.0,
            wheelbase=1.2,
            track_width=1.2,
        )
        result = calculate_swerve(params)
        # At Yaw = 0, robot faces +Y: FL is at top-left (-0.6, +0.6), FR at top-right (+0.6, +0.6)
        self.assertAlmostEqual(result.wheels["FL"].world_x, -0.6, places=6)
        self.assertAlmostEqual(result.wheels["FL"].world_y, 0.6, places=6)
        self.assertAlmostEqual(result.wheels["FR"].world_x, 0.6, places=6)
        self.assertAlmostEqual(result.wheels["FR"].world_y, 0.6, places=6)
        # Moving at Alpha = 0 points along +Y (90 deg in standard X-right, Y-up coords)
        self.assertAlmostEqual(result.wheels["FL"].world_angle_deg, 90.0, places=6)

    def test_yaw_rotates_robot_body_and_world_vectors(self):
        params = SwerveParams(
            velocity=0.0,
            alpha_deg=0.0,
            yaw_deg=90.0,
            omega=2.0,
            wheelbase=1.2,
            track_width=1.2,
        )
        result = calculate_swerve(params)
        # At Yaw = 0, FL is at (-0.6, +0.6). Rotated by +90 deg -> (-0.6, -0.6)
        fl = result.wheels["FL"]
        self.assertAlmostEqual(fl.world_x, -0.6, places=6)
        self.assertAlmostEqual(fl.world_y, -0.6, places=6)
        # World angle is 90 (for +Y zero) + local steering (135 deg) + yaw (90 deg) = 315 deg
        self.assertAlmostEqual(
            math.cos(math.radians(fl.world_angle_deg)),
            math.cos(math.radians(315.0)),
            places=6,
        )
        self.assertAlmostEqual(
            math.sin(math.radians(fl.world_angle_deg)),
            math.sin(math.radians(315.0)),
            places=6,
        )

    def test_yaw_does_not_change_movement_direction_field_centric(self):
        params = SwerveParams(
            velocity=5.0,
            alpha_deg=0.0,
            yaw_deg=30.0,
            omega=0.0,
            wheelbase=1.2,
            track_width=1.2,
        )
        result = calculate_swerve(params)
        for wheel in result.wheels.values():
            self.assertAlmostEqual(wheel.speed, 5.0, places=6)
            # Relative to the robot body (which is at +30 deg), wheel steers -30 deg
            self.assertAlmostEqual(wheel.steering_deg, -30.0, places=6)
            # In field/world space, Alpha = 0.0 deg points along +Y (90.0 deg on canvas)
            self.assertAlmostEqual(wheel.world_angle_deg, 90.0, places=6)


if __name__ == "__main__":
    unittest.main()
