"""Runtime joint measurements without Isaac/USD dependencies.

Every quaternion in this module is in ``(x, y, z, w)`` order. Body poses
are ``(x, y, z, qx, qy, qz, qw)``. USD's Gf quaternion objects must be
converted by the caller. Revolute coordinates are degrees; prismatic
coordinates use the same distance unit as the supplied body positions.
"""

import math


def _finite_values(values, count, label):
    try:
        result = [float(value) for value in values]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must contain {count} finite numbers") from exc
    if len(result) != count or not all(math.isfinite(v) for v in result):
        raise ValueError(f"{label} must contain {count} finite numbers")
    return result


def quat_normalize(quaternion):
    """Return a unit xyzw quaternion; reject a missing/zero orientation."""
    values = _finite_values(quaternion, 4, "quaternion")
    scale = max(abs(v) for v in values)
    if scale == 0.0:
        raise ValueError("quaternion has zero norm")
    scaled = [v / scale for v in values]
    norm = math.sqrt(sum(v * v for v in scaled))
    return [v / norm for v in scaled]


def quat_conjugate(quaternion):
    """Return the conjugate of an xyzw quaternion (inverse if unit)."""
    x, y, z, w = _finite_values(quaternion, 4, "quaternion")
    return [-x, -y, -z, w]


def quat_mul(left, right):
    """Hamilton product; ``left * right`` applies right, then left."""
    x0, y0, z0, w0 = _finite_values(left, 4, "left quaternion")
    x1, y1, z1, w1 = _finite_values(right, 4, "right quaternion")
    return [
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
    ]


def quat_rotate(quaternion, vector):
    """Rotate a three-vector by an xyzw quaternion."""
    q = quat_normalize(quaternion)
    v = _finite_values(vector, 3, "vector")
    return quat_mul(quat_mul(q, v + [0.0]), quat_conjugate(q))[:3]


def validate_pose(pose):
    """Validate a runtime xyz+xyzw transform and normalize its quaternion.

    A position-only sensor result, NaN/Inf, or zero quaternion is an error,
    never a zero-motion measurement.
    """
    values = _finite_values(pose, 7, "body pose")
    return values[:3] + quat_normalize(values[3:])


def _kind(kind):
    name = str(kind).lower()
    if name in ("hinge", "revolute", "revolutejoint", "angular"):
        return "hinge"
    if name in ("slider", "slide", "prismatic", "prismaticjoint", "linear"):
        return "slider"
    raise ValueError(f"unsupported joint kind: {kind!r}")


def _axis(axis):
    name = str(axis).upper()
    if name not in ("X", "Y", "Z"):
        raise ValueError(f"joint axis must be X, Y, or Z: {axis!r}")
    return ("X", "Y", "Z").index(name)


def joint_coordinate(kind, pose0, pose1, local_pos0, local_pos1,
                     local_rot0, local_rot1, axis):
    """Measure the child joint frame relative to the parent joint frame.

    Hinge output is the signed twist about ``axis`` in joint0's frame,
    wrapped to [-180, 180). It does not track complete revolutions; callers
    needing multi-turn angles must unwrap successive samples or use DOFs.
    Slider output is the joint-anchor separation projected on that axis.
    Common world translation/rotation of both bodies leaves either result
    unchanged. A hinge's 180-degree swing perpendicular to its twist axis
    makes twist undefined and raises ValueError.
    """
    joint_kind = _kind(kind)
    index = _axis(axis)
    p0, p1 = validate_pose(pose0), validate_pose(pose1)
    lp0 = _finite_values(local_pos0, 3, "local_pos0")
    lp1 = _finite_values(local_pos1, 3, "local_pos1")
    lr0, lr1 = quat_normalize(local_rot0), quat_normalize(local_rot1)
    frame0 = quat_normalize(quat_mul(p0[3:], lr0))
    frame1 = quat_normalize(quat_mul(p1[3:], lr1))

    if joint_kind == "slider":
        offset0, offset1 = quat_rotate(p0[3:], lp0), quat_rotate(p1[3:], lp1)
        separation = [p1[i] + offset1[i] - p0[i] - offset0[i] for i in range(3)]
        return quat_rotate(quat_conjugate(frame0), separation)[index]

    relative = quat_normalize(quat_mul(quat_conjugate(frame0), frame1))
    component, scalar = relative[index], relative[3]
    if math.hypot(component, scalar) < 1e-12:
        raise ValueError("hinge twist is undefined for this joint-frame orientation")
    degrees = math.degrees(2.0 * math.atan2(component, scalar))
    return (degrees + 180.0) % 360.0 - 180.0


def joint_constraint_error(kind, pose0, pose1, local_pos0, local_pos1,
                           local_rot0, local_rot1, axis):
    """Forbidden endpoint displacement (m) and rotation (deg), beyond the DOF."""
    kind, index = _kind(kind), _axis(axis)
    p0,p1 = validate_pose(pose0),validate_pose(pose1)
    f0 = quat_normalize(quat_mul(p0[3:],quat_normalize(local_rot0)))
    f1 = quat_normalize(quat_mul(p1[3:],quat_normalize(local_rot1)))
    a0 = quat_rotate(p0[3:],_finite_values(local_pos0,3,"local_pos0"))
    a1 = quat_rotate(p1[3:],_finite_values(local_pos1,3,"local_pos1"))
    gap = [p1[i]+a1[i]-p0[i]-a0[i] for i in range(3)]
    unit = [0.,0.,0.]; unit[index] = 1.
    axis0,axis1 = quat_rotate(f0,unit),quat_rotate(f1,unit)
    if kind == "hinge":
        distance = math.hypot(*gap)
        cosine = max(-1.,min(1.,sum(a*b for a,b in zip(axis0,axis1))))
        angle = math.degrees(math.acos(cosine))
    else:
        along = sum(a*b for a,b in zip(gap,axis0))
        distance = math.hypot(*[gap[i]-along*axis0[i] for i in range(3)])
        cosine = min(1.,abs(sum(a*b for a,b in zip(f0,f1))))
        angle = math.degrees(2*math.acos(cosine))
    return {"position_m":distance,"angle_deg":angle}


def _limit(value, fallback, label):
    if value is None:
        return fallback
    try:
        result = float(value)
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f"{label} must be numeric or None") from exc
    if math.isnan(result):
        raise ValueError(f"{label} cannot be NaN")
    return result


def target_plan(kind, lo, hi):
    """Choose an interior reference and two legal targets around it.

    Default amplitude is 20 degrees for hinges or 0.02 distance units for
    sliders. Finite ranges cap it at 20% of their width. Narrow ranges use
    their midpoint. Wider ranges keep the reference as close to zero as
    possible, with two amplitudes of clearance from either finite boundary.
    One-sided ranges also place the reference two amplitudes inside their
    finite boundary. This avoids unnecessary multi-turn hinge warm-up.
    An equal, reversed, unusably narrow, or invalid range raises ValueError.
    """
    joint_kind = _kind(kind)
    lower = _limit(lo, -math.inf, "lower limit")
    upper = _limit(hi, math.inf, "upper limit")
    if lower == upper:
        raise ValueError("joint is locked: lower and upper limits are equal")
    if lower > upper or lower == math.inf or upper == -math.inf:
        raise ValueError("joint limits are reversed or invalid")

    amplitude = 20.0 if joint_kind == "hinge" else 0.02
    if math.isfinite(lower) and math.isfinite(upper):
        width = upper - lower
        amplitude = min(amplitude, 0.2 * width)
        center = lower / 2.0 + upper / 2.0
        if width > 5.0 * amplitude:
            center = min(max(0.0, lower + 2.0 * amplitude), upper - 2.0 * amplitude)
    elif math.isfinite(lower):
        center = lower + 2.0 * amplitude
    elif math.isfinite(upper):
        center = upper - 2.0 * amplitude
    else:
        center = 0.0
    positive, negative = center + amplitude, center - amplitude
    if not (all(math.isfinite(v) for v in (center, amplitude, positive, negative))
            and amplitude > 0.0
            and lower <= negative < center < positive <= upper):
        raise ValueError("joint limits cannot represent distinct legal test targets")
    return {"center": center, "amplitude": amplitude,
            "positive": positive, "negative": negative}
