# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""URDF post-processing helpers shared across simulator and MoveIt launches.

Lives in ``genie_sim_robot_model`` because that package is a dependency of
both ``genie_sim_bringup`` (which publishes the simulator's URDF on
``/robot_description``) and ``genie_sim_moveit`` (which loads its own URDF
through ``MoveItConfigsBuilder``).  Keeping the helper here avoids a
``bringup → moveit`` or ``moveit → bringup`` dependency loop while letting
both call sites apply the same transformation to the URDF string.
"""

from __future__ import annotations


def pad_urdf_joint_limits(urdf_text: str, pad_revolute_rad: float, pad_prismatic_m: float) -> str:
    """Widen ``<limit lower=.. upper=..>`` on every URDF joint by a small pad.

    Why: MuJoCo's joint limits are soft (Baumgarte-stabilised impulse
    constraint), so the simulated joint can drift past the URDF limit by
    a fraction of a degree under heavy contact load.  MoveIt 2's
    ``CheckStartStateBounds`` adapter then rejects planning requests
    with errors like::

        Joint 'idx61_arm_r_joint1' from the starting state is outside
        bounds by: [-3.07236] should be in the range [-3.0718] [3.0718].

    The overshoot is sub-mrad in practice -- well below anything the
    planner needs to respect -- but the rejection blocks every grasp
    attempt that lands the arm near a hard stop.  MoveIt's
    ``start_state_max_bounds_error`` only feeds the *Fix*StartStateBounds
    adapter; ``Check``StartStateBounds reads URDF limits directly.

    Cleanest fix: widen the URDF that MoveIt reads.  The simulator keeps
    enforcing the original (slightly tighter) limit via its own joint
    classifier, and MoveIt sees a URDF whose limit is the sim limit + a
    small pad that absorbs the soft-constraint drift.  Pad magnitude is
    intentionally small (1.0e-2 rad ~ 0.57 deg revolute, 1.0e-3 m = 1 mm
    prismatic) so generated trajectories stay within the physical range
    -- this is a tolerance widening, not a limit relax.

    Both ``revolute`` and ``continuous`` joints are skipped if the
    declared range is already 2*pi or wider (a continuous joint's URDF
    may carry placeholder limits but is effectively unlimited).

    Returns the URDF string unchanged if XML parsing fails -- better to
    surface that as a launch error downstream than to half-edit the XML.
    """
    try:
        import xml.etree.ElementTree as ET  # noqa: PLC0415
    except ImportError:
        return urdf_text
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        return urdf_text

    two_pi = 6.2831853  # rad
    n_widened = 0
    for joint in root.iter("joint"):
        jtype = (joint.get("type") or "").strip().lower()
        if jtype not in ("revolute", "prismatic", "continuous"):
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        lower_str = limit.get("lower")
        upper_str = limit.get("upper")
        if lower_str is None or upper_str is None:
            continue
        try:
            lower = float(lower_str)
            upper = float(upper_str)
        except ValueError:
            continue
        if jtype in ("revolute", "continuous"):
            if (upper - lower) >= two_pi:
                continue
            pad = pad_revolute_rad
        else:
            pad = pad_prismatic_m
        if pad <= 0.0:
            continue
        limit.set("lower", f"{lower - pad:.9g}")
        limit.set("upper", f"{upper + pad:.9g}")
        n_widened += 1

    if n_widened == 0:
        return urdf_text
    out = ET.tostring(root, encoding="unicode")
    # Preserve the <?xml ... ?> prolog if the input had one -- keeps the
    # output byte-comparable to what xacro emits when no pad is applied.
    stripped = urdf_text.lstrip()
    if stripped.startswith("<?xml"):
        end = stripped.find("?>")
        if end != -1:
            out = stripped[: end + 2] + "\n" + out
    return out


def inject_base_footprint_prismatic(
    urdf_text: str,
    *,
    base_link_name: str = "base_link",
    footprint_link_name: str = "base_footprint",
    joint_name: str = "base_footprint_to_base_link",
    upper_limit_m: float = 0.50,
) -> str:
    """Insert ``base_footprint`` and a passive prismatic Z joint above ``base_link``.

    Why: MoveIt's planar virtual_joint stores only (x, y, theta); base_link's
    z is forced to 0 in RobotState.  We need MoveIt's view of base_link to
    track the simulator's actual ride height (chassis bobbing, traversing
    a small platform, wheel deformation) so that collision-relative-to-
    world-frame manipulation is consistent.

    Without modifying the canonical URDF that the simulator's assemble
    pipeline consumes, this helper rewrites the MoveIt-side URDF string
    at launch time to insert a synthetic chain::

        odom (planar virtual_joint, x/y/theta)
         -> base_footprint  (massless link, ground-projected)
         -> base_link       (real chassis link, was attached to odom directly)
              via prismatic joint along z

    The simulator publishes the prismatic joint's value on ``/joint_states``
    each tick (= ``body_q[base_link].z``), so RSP applies the live ride
    height to MoveIt's RobotState transparently.  The SRDF marks the joint
    as ``<passive_joint>`` so the planner never tries to vary z.

    The simulator never sees ``base_footprint`` or the prismatic joint --
    its URDF / robot.usda is untouched.  This is purely a MoveIt-side
    re-parenting hack, applied via the same launch-time mutation pattern
    as :func:`pad_urdf_joint_limits`.

    Idempotent: if ``base_footprint`` is already in the URDF (e.g. somebody
    later promotes it into the canonical xacro), this is a no-op.

    Returns the URDF string unchanged if XML parsing fails or
    ``base_link`` isn't found -- better to surface that as a launch error
    downstream than to half-edit the XML.
    """
    try:
        import xml.etree.ElementTree as ET  # noqa: PLC0415
    except ImportError:
        return urdf_text
    try:
        root = ET.fromstring(urdf_text)
    except ET.ParseError:
        return urdf_text

    # Idempotency: bail if base_footprint already exists.
    for link in root.iter("link"):
        if link.get("name") == footprint_link_name:
            return urdf_text

    # Sanity: target base_link has to exist.
    base_link_present = any(link.get("name") == base_link_name for link in root.iter("link"))
    if not base_link_present:
        return urdf_text

    # Build the new link element.
    fp_link = ET.Element("link", {"name": footprint_link_name})
    fp_inertial = ET.SubElement(fp_link, "inertial")
    ET.SubElement(fp_inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(fp_inertial, "mass", {"value": "0.001"})
    ET.SubElement(
        fp_inertial,
        "inertia",
        {"ixx": "1e-6", "iyy": "1e-6", "izz": "1e-6", "ixy": "0", "ixz": "0", "iyz": "0"},
    )

    # Build the prismatic joint connecting base_footprint -> base_link.
    # IMPORTANT: parent IS base_footprint, child IS base_link, so MoveIt's
    # kinematic tree becomes  odom -> base_footprint -> (prismatic) -> base_link.
    fp_joint = ET.Element("joint", {"name": joint_name, "type": "prismatic"})
    ET.SubElement(fp_joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(fp_joint, "parent", {"link": footprint_link_name})
    ET.SubElement(fp_joint, "child", {"link": base_link_name})
    ET.SubElement(fp_joint, "axis", {"xyz": "0 0 1"})
    # Passive joint: effort=0, velocity=0 (planner can't drive it; SRDF
    # also tags it <passive_joint> so it never lands in any planning
    # group's active variable list).  Position bound = [0, upper_limit_m]
    # is a generous chassis ride-height envelope.
    ET.SubElement(
        fp_joint,
        "limit",
        {"lower": "0.0", "upper": f"{upper_limit_m:.6f}", "effort": "0", "velocity": "0"},
    )

    # Insert the new link + joint as the FIRST children of <robot> so they
    # land before base_link's first reference (any joint whose parent or
    # child is base_link).  URDF doesn't strictly require ordering, but
    # some parsers walk top-down and surface friendlier errors when the
    # parent appears first.
    root.insert(0, fp_joint)
    root.insert(0, fp_link)

    out = ET.tostring(root, encoding="unicode")
    stripped = urdf_text.lstrip()
    if stripped.startswith("<?xml"):
        end = stripped.find("?>")
        if end != -1:
            out = stripped[: end + 2] + "\n" + out
    return out
