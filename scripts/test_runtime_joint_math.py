"""Pure sensor/math regression tests; no Isaac runtime is imported."""

import math
import unittest

from runtime_joint_math import (
    joint_coordinate, quat_conjugate, quat_mul, quat_normalize, quat_rotate,
    target_plan, validate_pose, joint_constraint_error,
)


IDENTITY = [0.0, 0.0, 0.0, 1.0]
ORIGIN = [0.0, 0.0, 0.0]


def rotation(axis, degrees):
    half = math.radians(degrees) / 2.0
    return [v * math.sin(half) for v in axis] + [math.cos(half)]


def pose(position=ORIGIN, quaternion=IDENTITY):
    return list(position) + list(quaternion)


def common_transform(body_pose, translation, quaternion):
    position = quat_rotate(quaternion, body_pose[:3])
    return pose([position[i] + translation[i] for i in range(3)],
                quat_mul(quaternion, body_pose[3:]))


class QuaternionTests(unittest.TestCase):
    def assertVectorAlmostEqual(self, actual, expected):
        for a, b in zip(actual, expected):
            self.assertAlmostEqual(a, b, places=10)

    def test_rotate_and_inverse(self):
        q = rotation([0, 0, 1], 90)
        self.assertVectorAlmostEqual(quat_rotate(q, [1, 0, 0]), [0, 1, 0])
        self.assertVectorAlmostEqual(quat_mul(q, quat_conjugate(q)), IDENTITY)

    def test_normalization_handles_large_values(self):
        self.assertVectorAlmostEqual(quat_normalize([1e308] * 4), [0.5] * 4)

    def test_pose_validation(self):
        self.assertEqual(validate_pose([1, 2, 3, 0, 0, 0, 2]), [1, 2, 3, 0, 0, 0, 1])
        for invalid in ([1, 2, 3], [0] * 7, [0, 0, 0, 0, 0, 0, math.nan],
                        [math.inf, 0, 0, 0, 0, 0, 1], None):
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                validate_pose(invalid)


class JointCoordinateTests(unittest.TestCase):
    def measure(self, kind, p0, p1, lp0=ORIGIN, lp1=ORIGIN,
                lr0=IDENTITY, lr1=IDENTITY, axis="Z"):
        return joint_coordinate(kind, p0, p1, lp0, lp1, lr0, lr1, axis)

    def test_positive_and_negative_z(self):
        for degrees in (-35, 20):
            with self.subTest(degrees=degrees):
                result = self.measure("RevoluteJoint", pose(), pose(quaternion=rotation([0, 0, 1], degrees)))
                self.assertAlmostEqual(result, degrees)

    def test_quaternion_sign_equivalence(self):
        q0 = rotation([0, 1, 0], 55)
        q1 = quat_mul(q0, rotation([0, 0, 1], 35))
        for s0 in (-1, 1):
            for s1 in (-1, 1):
                result = self.measure("hinge", pose(quaternion=[s0 * v for v in q0]),
                                      pose(quaternion=[s1 * v for v in q1]))
                self.assertAlmostEqual(result, 35)

    def test_common_parent_child_rotation_leaves_coordinate_unchanged(self):
        p0 = pose([2, -1, 3], rotation([0, 1, 0], 40))
        p1 = pose([2, -1, 3], quat_mul(p0[3:], rotation([0, 0, 1], -25)))
        common = quat_mul(rotation([1, 0, 0], 75), rotation([0, 0, 1], 110))
        shifted = [common_transform(p, [8, 9, -2], common) for p in (p0, p1)]
        self.assertAlmostEqual(self.measure("hinge", *shifted), -25)

    def test_local_joint_frames_and_negative_world_axis(self):
        # The joint's +X points along world -Z; the bodies have unrelated bases.
        frame = rotation([0, 1, 0], 90)
        q0 = rotation([1, 0, 0], 47)
        q1_initial = rotation([0, 0, 1], 63)
        lr0 = quat_mul(quat_conjugate(q0), frame)
        lr1 = quat_mul(quat_conjugate(q1_initial), frame)
        q1 = quat_mul(rotation([0, 0, -1], 30), q1_initial)
        self.assertAlmostEqual(self.measure("hinge", pose(quaternion=q0), pose(quaternion=q1),
                                            lr0=lr0, lr1=lr1, axis="X"), 30)

    def test_slider_uses_parent_frame_and_both_anchor_offsets(self):
        q0 = rotation([0, 0, 1], 90)
        # Parent local anchor [1,0,0] is at world [0,1,0]. Child anchor
        # is at world [0,1.04,0]; parent local +X is world +Y.
        p0 = pose(quaternion=q0)
        p1 = pose([0, 0.94, 0])
        result = self.measure("PrismaticJoint", p0, p1, [1, 0, 0], [0, 0.1, 0], axis="X")
        self.assertAlmostEqual(result, 0.04)
        common = rotation([1, 0, 0], 120)
        shifted = [common_transform(p, [3, -7, 2], common) for p in (p0, p1)]
        self.assertAlmostEqual(self.measure("slider", *shifted, [1, 0, 0], [0, 0.1, 0], axis="X"), 0.04)

    def test_slider_local_joint_rotation(self):
        self.assertAlmostEqual(self.measure("slider", pose(), pose([0, 0, -0.02]),
                                            lr0=rotation([0, 1, 0], 90), axis="X"), 0.02)

    def test_twist_excludes_swing(self):
        relative = quat_mul(rotation([1, 0, 0], 45), rotation([0, 0, 1], 30))
        self.assertAlmostEqual(self.measure("hinge", pose(), pose(quaternion=relative)), 30)

    def test_undefined_twist_and_bad_axis_are_errors(self):
        with self.assertRaises(ValueError):
            self.measure("hinge", pose(), pose(quaternion=rotation([1, 0, 0], 180)))
        with self.assertRaises(ValueError):
            self.measure("hinge", pose(), pose(), axis="bad")


class TargetPlanTests(unittest.TestCase):
    def test_one_sided_finite_ranges(self):
        for kind, lo, hi, expected_amplitude in (
                ("hinge", 0, 111, 20), ("hinge", -98, 0, 19.6),
                ("slider", 0, 0.08, 0.016), ("hinge", -1, 1, 0.4)):
            with self.subTest(kind=kind, lo=lo, hi=hi):
                plan = target_plan(kind, lo, hi)
                self.assertAlmostEqual(plan["amplitude"], expected_amplitude)
                self.assertLess(lo, plan["negative"])
                self.assertLess(plan["negative"], plan["center"])
                self.assertLess(plan["center"], plan["positive"])
                self.assertLess(plan["positive"], hi)

    def test_half_bounded_and_unbounded_ranges(self):
        for lo, hi in ((0, None), (None, 0), (-math.inf, math.inf)):
            plan = target_plan("hinge", lo, hi)
            self.assertAlmostEqual(plan["positive"] - plan["center"], 20)
            self.assertAlmostEqual(plan["center"] - plan["negative"], 20)
            if lo is not None:
                self.assertGreater(plan["negative"], lo)
            if hi is not None:
                self.assertLess(plan["positive"], hi)

    def test_wide_ranges_stay_near_zero(self):
        # C1000 lid_force_knob must not warm up to its 1718-degree midpoint.
        for lo, hi, expected_center in ((0, 3437.74677, 40),
                                        (-3437.74677, 0, -40),
                                        (-10313, 10313, 0)):
            with self.subTest(lo=lo, hi=hi):
                plan = target_plan("hinge", lo, hi)
                self.assertEqual(plan["center"], expected_center)
                self.assertEqual(plan["amplitude"], 20)
                self.assertGreater(plan["negative"], lo)
                self.assertLess(plan["positive"], hi)
        self.assertAlmostEqual(target_plan("slider", 0, 0.08)["center"], 0.04)
        self.assertAlmostEqual(target_plan("hinge", -98, 0)["center"], -49)

    def test_locked_reversed_and_invalid_ranges_are_errors(self):
        for lo, hi in ((0, 0), (1, -1), (math.nan, 1), (math.inf, None), (None, -math.inf)):
            with self.subTest(lo=lo, hi=hi), self.assertRaises(ValueError):
                target_plan("hinge", lo, hi)


class ConstraintTests(unittest.TestCase):
    def test_hinge_allows_twist_but_not_anchor_separation(self):
        good = joint_constraint_error("hinge",pose(),pose(quaternion=rotation([0,0,1],25)),
            ORIGIN,ORIGIN,IDENTITY,IDENTITY,"Z")
        self.assertAlmostEqual(good["position_m"],0)
        self.assertAlmostEqual(good["angle_deg"],0)
        bad = joint_constraint_error("hinge",pose(),pose([.001,0,0]),
            ORIGIN,ORIGIN,IDENTITY,IDENTITY,"Z")
        self.assertAlmostEqual(bad["position_m"],.001)

    def test_slider_allows_axial_motion_but_not_shear_or_rotation(self):
        good = joint_constraint_error("slider",pose(),pose([.05,0,0]),
            ORIGIN,ORIGIN,IDENTITY,IDENTITY,"X")
        self.assertAlmostEqual(good["position_m"],0)
        bad = joint_constraint_error("slider",pose(),pose([.05,.002,0],rotation([0,0,1],10)),
            ORIGIN,ORIGIN,IDENTITY,IDENTITY,"X")
        self.assertAlmostEqual(bad["position_m"],.002)
        self.assertAlmostEqual(bad["angle_deg"],10)


if __name__ == "__main__":
    unittest.main()
