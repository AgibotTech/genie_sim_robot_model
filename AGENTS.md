# genie_sim_robot_model

Robot description package — xacro/URDF sources and pre-baked USD assets for
all robots supported by the GenieSim stack.

Source: [source/geniesim_ros/src/ros_ws/src/genie_sim_robot_model/](.)
License: [Mozilla Public License Version 2.0](../../../../LICENSE) (per-robot upstream licenses — see [`THIRD_PARTY_NOTICES`](THIRD_PARTY_NOTICES))
Third-party notices: [THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES)

**Maintenance contract**: when you add a robot, update `robots/`, `xacro/`,
`THIRD_PARTY_NOTICES`, and this file in the same diff.

---

## Support tiers

The GenieSim stack actively develops and validates **AgiBot Genie G2 only**.
Every other robot description in `robots/` is community-grade reference
material, kept buildable but not exercised against the full physics /
launch / fold-task pipelines.

| Tier | Robots | Meaning |
|---|---|---|
| **Tier 1 — actively supported** | `genie/g2/*` (G2 with `crs` / `crsB` arm × `omnipicker` / `swiftpicker` gripper) | Maintained against every release. Featured in the launcher and scene yamls (`scene_flat_g2_*`, `launcher_newton_*`). Physics tuning, contact compliance, mimic constraints, mobile-base pin/free behaviour are all validated on G2. Bug reports get triaged on this hardware first. |
| **Tier 2 — reference only** | `agilex/aloha`, `agilex/piper`, `arx/x5`, `arx/acone`, `franka/fr3`, `universal_robots/ur5` | URDF/xacro sources kept correct and the URDF→USD pipeline expected to import them. Scene yamls and physics_params entries may be incomplete or stale. Drive gains, contact params, and end-effector behaviour are **not** continuously validated. Treat them as starting points — expect to tune. |

Concretely, if you're filing an issue or contributing a feature:

* On G2: the maintainers will reproduce against the canonical
  `scene_flat_g2_sp_*.yaml` scenes and the canonical
  `launcher_newton_{mjwarp,mjvbd,fsvbd,avbd}.yaml` launchers, and any
  divergence from documented behaviour is treated as a bug.
* On a Tier-2 robot: PRs welcome, but the maintainers will not by
  default reproduce on that platform. Include a complete reproducer
  (scene yaml, launcher yaml, exact command, expected vs observed) and
  the bar for "we'll fix this" is higher.

If you need Tier-1 support for a non-G2 platform, the path is to
contribute a continuously-runnable scene + launcher pair that the CI /
release pipeline can include in its smoke tests, and to maintain it.

---

## Layout

```
genie_sim_robot_model/
├── xacro/
│   └── robot.xacro                ← universal entry point (dispatches by robot_model)
├── robots/
│   ├── agilex/                    ← AgileX Robotics (aloha, piper)            [Tier 2]
│   ├── arx/                       ← ARX Robotics (x5, acone)                  [Tier 2]
│   ├── franka/                    ← Franka Robotics (fr3)                     [Tier 2]
│   ├── genie/                     ← AgiBot Genie mobile manipulation platform [Tier 1]
│   │   └── g2/                    ← G2 variants: crs/crsB arm × omnipicker/swiftpicker gripper
│   └── universal_robots/          ← Universal Robots (ur5)                    [Tier 2]
├── scripts/
│   ├── normalize_obj_names.py     ← OBJ object-name normalizer (mesh-developer tool)
│   └── diagnose_urdf.py           ← URDF/xacro authoring diagnose+fix tool
├── urdf/                          ← generated URDF outputs (not committed)
├── rviz/                          ← per-robot RViz2 configs
└── launch/                        ← robot_state_publisher launch files
```

---

## Mesh-developer tools

### `scripts/normalize_obj_names.py` — defeat 6.0 converter mesh de-duplication

The Isaac Sim 6.0 URDF→USD converter (`urdf_usd_converter._impl.mesh`) keys
each authored mesh prim under `Geometries/<name>` where `<name>` is the
.obj file's `o <name>` directive, and de-duplicates by name across files.
DCC tools (3ds Max, Blender, Maya) commonly emit generic default object
names — `Cylinder001`, `Box001`, `s`, `2`, `1` — that collide aggressively
across a multi-link robot. When a collision happens, the first occurrence
wins and the rest get a `_N` suffix or are silently dropped, leaving the
assembled robot USD with **missing or swapped link visuals**.

ARX acone is the canonical case: 14+ visual meshes share four distinct
internal names. Without normalization the assembled USD is missing
`arm_r_*` and half the gripper visuals. UR5 doesn't trip it because its
`.dae` meshes carry per-link names baked in by the DCC tool.

**The tool** rewrites every `.obj` in place to carry exactly one
`o <file_stem>` directive (file stem = filename without `.obj`) and zero
`g` directives. Vertex/normal/texture/face/material lines are preserved
unmodified. The result is one unique object name per file, derived
deterministically from the filename → one unique
`Geometries/<file_stem>` prim in the converted USD, no cross-file
collisions. The tool is idempotent.

#### Usage

```bash
# Diagnose only — list every .obj, report cross-file name collisions
# and which files would be rewritten. No file is modified.
python3 scripts/normalize_obj_names.py --dry-run [PATH ...]

# Auto-fix — rewrite every .obj in place. Re-run after each batch of
# mesh updates from the DCC tool.
python3 scripts/normalize_obj_names.py [PATH ...]
```

If no PATH is given, the tool scans the current working directory
recursively. Pass directories or individual `.obj` files to limit scope.

`--dry-run` exits non-zero if any file needs normalization (so it doubles
as a CI gate); the auto-fix run exits zero if all writes succeed.

#### Mesh-developer workflow

1. Export new `.obj` meshes from the DCC tool.
2. Drop them under `robots/<vendor>/<robot>/meshes/`.
3. Run `python3 scripts/normalize_obj_names.py --dry-run robots/<vendor>` to
   see what's about to change.
4. Run `python3 scripts/normalize_obj_names.py robots/<vendor>` to apply
   the rewrite.
5. Commit the normalized `.obj` files.
6. Delete any stale `assets/scenes/<scene_stem>/` cache so the next launch
   re-runs `assemble_robot.py` against the fixed meshes.

The `assemble_robot.py` URDF→USD pipeline does **not** carry a runtime
fallback for non-normalized .obj files — that responsibility lives with
the mesh-source-of-truth here.

### `scripts/diagnose_urdf.py` — URDF/xacro authoring diagnose + fix

Detects authoring problems in URDF/xacro source that hurt PhysX
simulation quality and (where possible) auto-fixes them in place.

#### Checks

**1. Missing `<inertial>` on a rigid-body link (auto-fixable)** — PhysX
needs mass + diagonal inertia for every rigid body. When a URDF link
becomes a `RigidBodyAPI` prim in the converted USD AND lacks
`<inertial>`, the simulator falls back to "small sphere with negative
mass" and emits the `[omni.physx.plugin] possibly invalid inertia
tensor of {1.0, 1.0, 1.0} and a negative mass` warning at startup. The
dynamics under that fallback are wrong — drive wheels may not push the
body, contact responses may be off.

The 6.0 converter applies `RigidBodyAPI` to **every** non-ghost-root
link, so the rule flags any non-root link that has **neither
`<inertial>` nor `<collision>`** — including pure kinematic frames (TF
anchors, branching mount frames, sensor frames). They are NOT exempt: a
fixed-joint-connected frame with no mass still becomes a massless rigid
body and warns. Three cases are excluded:

* **The URDF root link.** KDL (used by `robot_state_publisher` and
  MoveIt) requires the root link to be massless. Putting `<inertial>`
  on the root produces "Root link X has inertial properties..."
  warnings and breaks downstream KDL consumers. Detected as the unique
  link that is not the `<child>` of any `<joint>`. (The root's own
  massless-rigid-body warning is the documented cost of the
  massless-root + `_inertia`-sibling pattern — see
  [Non-ghost root link pattern](#non-ghost-root-link-pattern).)
* **`<X>` + `<X>_inertia` sibling pattern.** Universal Robots (and
  several other vendors) put the actual mass on a separate `*_inertia`
  child link connected by a fixed joint, leaving the named link as a
  pure kinematic frame for clean joint-axis math. This is a
  community/vendor convention rather than a formal REP. Injecting on
  the named link duplicates mass authored on the inertia partner.
  Detected by literal sibling name lookup.
* **Links with a `<collision>`.** PhysX auto-computes mass from the
  collision shape's volume × density, so a missing `<inertial>` is not
  a problem for them.

What's left after these exclusions is the set of non-root links that
lack both `<inertial>` and `<collision>` — these are the real PhysX
warnings. The tool inserts:

```xml
<inertial>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <mass value="0.001"/>
  <inertia ixx="1e-6" iyy="1e-6" izz="1e-6" ixy="0" ixz="0" iyz="0"/>
</inertial>
```

Six orders of magnitude below typical link masses — silences the
warning on visual-only frame links (e.g. mounting brackets that have a
visual mesh but no contact role) without affecting dynamics. **For
load-bearing links the operator must edit the placeholder to real
values.** For instance, the aloha `chassis_link` is ~30 kg with a ~1 m³
bounding box; leaving it at `0.001 kg` means the drive wheels can't
move it.

Self-closing forms like `<link name="X"/>` are expanded into a full
`<link>...</link>` block before injection so the new `<inertial>` can
sit inside as a child.

**2. URDF ghost root (detect-only)** — when the URDF root link has no
`<visual>`, no `<collision>`, AND no `<inertial>`, the Isaac Sim 6.0
URDF→USD converter takes its `is_ghost_link` branch and authors
`body0 = default_prim` for **every** joint, including fixed joints.
PhysX cannot construct an articulation from such USD: every joint
appears anchored to the world through a non-RigidBodyAPI prim.

The tool can't auto-fix this because:
* KDL forbids `<inertial>` on the root (breaks `robot_state_publisher`),
  ruling out the simplest fix.
* Adding `<visual>`/`<collision>` to the root changes rendering or
  contact behavior and the operator should pick what's appropriate.

The fix is to restructure the URDF — see
[Design patterns → Non-ghost root link](#non-ghost-root-link-pattern)
for the canonical solution used across all robots in this package.

**3. Wheel without `<cylinder>` collision (detect-only)** — wheel-like
links (name contains `wheel` or `tire`, case-insensitive) benefit from
a primitive `<cylinder>` collider rather than a triangle-mesh collider.
Cylinder vs ground-plane collision is ~10× faster than mesh-vs-mesh and
numerically much more stable for sustained contact — the dominant
workload on a mobile platform. The 6.0 default-collision policy
auto-converts authored mesh colliders to convex hull, which is
workable but inferior.

The diagnose tool flags wheels lacking a `<cylinder>` collision so the
operator authors one explicitly. **This check is detect-only**: the
tool can't infer wheel radius/length from URDF text alone — measure
the wheel mesh and author the cylinder by hand. Convention is the
cylinder's `length` axis aligned with the wheel's rotation axis.

Castor wheels and mounts are deliberately **not** flagged — castor
wheel links often live under generic names (`chassis_*_link1`) that
the regex can't disambiguate from non-wheel links without false
positives. Flag those by renaming the link to include `wheel` in its
name.

#### Operates on xacro source

The tool defaults to fixing **xacro** rather than the generated URDF.
Re-generating the URDF from xacro would wipe a URDF-only fix; fixing
the xacro means every regenerated URDF picks up the change.

For xacro-parametric link names (`<link name="arm_${prefix}_link1"/>`
inside a `<xacro:macro>`), the inertial detector still flags them but
the auto-fix can't string-match the templated name reliably. Those
cases are reported as "could not be patched" — fix them by editing the
macro body directly to template an `<inertial>` block.

#### Usage

```bash
# Diagnose only — exits 1 if any problem is found.
python3 scripts/diagnose_urdf.py --dry-run [PATH ...]

# Auto-fix the auto-fixable problems (currently: missing <inertial>).
python3 scripts/diagnose_urdf.py [PATH ...]
```

PATH may be a file (`.xacro` / `.urdf.xacro` / `.urdf`) or a directory
(recursive scan). Default: current directory (recursive).

Exits non-zero on `--dry-run` if any problem exists. The default-mode
exit code is non-zero **also when detect-only problems remain after
auto-fix** so wheel-cylinder gaps stay visible in CI.

#### Mesh-developer workflow

1. Add or modify a `<link>` in the xacro.
2. Run `python3 scripts/diagnose_urdf.py --dry-run robots/<vendor>` to
   see all flagged problems (inertial + ghost-root + wheel).
3. Run `python3 scripts/diagnose_urdf.py robots/<vendor>` to insert
   placeholder `<inertial>` blocks for the auto-fixable cases.
4. **Edit each placeholder for load-bearing links** (chassis, body
   shells, anything that needs real dynamics). Visual-only frame links
   can keep the placeholder.
5. For each wheel flagged as missing `<cylinder>` collision, author the
   cylinder by hand — measure or estimate radius and length from the
   wheel mesh.
6. For each ghost-root link flagged, restructure the URDF — typical
   options: add a small dummy `<visual>` to the root, or remove the
   ghost root entirely and start the chain at the next link.
7. Commit the xacro with the inertial blocks, wheel cylinders, and any
   ghost-root restructuring.
8. Delete any stale `assets/scenes/<scene_stem>/` cache and relaunch.

The `assemble_robot.py` URDF→USD pipeline does **not** carry runtime
fallbacks for these — that responsibility lives here.

---

## xacro entry point

`xacro/robot.xacro` is the **single entry point** for all robots. It dispatches
by the `robot_model` xacro arg. Additional args (`body`, `arm`, `gripper`,
`variant`) select sub-configurations within a robot family.

Called by `assemble_robot.py` in `genie_sim_engine` via:
```python
xacro.process_file("share/genie_sim_robot_model/xacro/robot.xacro",
                   mappings={"robot_model": ..., "arm": ..., "body": ..., "gripper": ...})
```

Any string key under `robot.robot_source.urdf:` in the scene YAML (except
reserved keys `xacro_relpath`, `mimic`) is forwarded as an additional xacro
mapping.

Reserved xacro mappings (set by the launch system):

| Key | Source | Purpose |
|---|---|---|
| `robot_model` | scene YAML / CLI | Selects the robot family |
| `body` | scene YAML / CLI | Body variant |
| `arm` | scene YAML / CLI | Arm variant |
| `gripper` | scene YAML / CLI | Gripper choice (consistent arg name across piper / G2 / UR5) |

---

## Adding a robot

1. Create `robots/<robot_name>/` with xacro, meshes, and collision geometry.
2. Add a dispatch branch in `xacro/robot.xacro`.
3. Add third-party license info to `THIRD_PARTY_NOTICES` if the robot
   description originates from an upstream vendor.
4. **Run mesh-developer tools** if any link references `.obj` meshes:
   - `.obj` → `python3 scripts/normalize_obj_names.py robots/<vendor>`
   - `.fbx` must be converted to `.dae` beforehand using an external tool
     (e.g. Blender, Maya). The URDF→USD pipeline only supports
     `.stl` / `.obj` / `.dae`. Update the xacro to reference `.dae`.
   - `.dae`/`.stl`-only robots need no mesh-dev preprocessing.
5. **Run the URDF/xacro diagnose tool** to verify every link has an
   ``<inertial>`` block:
   `python3 scripts/diagnose_urdf.py --dry-run robots/<vendor>`. If
   anything is flagged, run without `--dry-run` to inject placeholders,
   then edit the placeholders for load-bearing links to real values.
6. Test: `xacro xacro/robot.xacro robot_model:=<name>` must produce valid URDF.
7. Test the URDF→USD pipeline:
   `ros2 launch genie_sim_bringup app.launch.py scene:=<scene_with_new_robot> ...`

---

## URDF→USD pipeline (upstream context)

This package is a **pure description provider** — it does not run the importer.
The URDF→USD conversion is owned by `genie_sim_engine/scripts/assemble_robot.py`.
See [genie_sim_engine/AGENTS.md](../genie_sim_engine/AGENTS.md) for the full
pipeline description.

Key contract: `fix_base` in the scene YAML's `robot.robot_source` controls
whether the URDF root is welded to world (`true`, default, for arm-only robots)
or left as a floating 6-DOF root (`false`, required for mobile platforms so
wheel friction can translate the chassis).

---

## Design patterns

Conventions every robot in this package follows. New robots SHOULD
adopt these; deviations need a comment in the xacro explaining why.

### Non-ghost root link pattern

**Problem**

The Isaac Sim 6.0 URDF→USD converter
(`urdf_usd_converter._impl.link.physics_joints`) branches on a single
boolean — whether the URDF root link has any `<inertial>`, `<visual>`,
or `<collision>` child:

```python
is_ghost_link = (root_link.inertial is None
                 and len(root_link.visuals) == 0
                 and len(root_link.collisions) == 0)
```

When `is_ghost_link=True` (the URDF root is a pure kinematic frame,
which is the standard ROS convention for `base_link` / `world` / etc.),
the converter takes a Newton-only authoring branch: every joint —
including every fixed joint deep in the kinematic tree — gets
`body0 = default_prim` (the articulation root Xform), not the actual
parent link. PhysX cannot construct an articulation from such USD:
every joint appears anchored to the world origin simultaneously, the
robot collapses through the floor on the first physics step, and no
runtime patch can fix the topology after the fact.

This is documented in detail in
[genie_sim_engine/AGENTS.md → URDF/xacro authoring as the source of truth](../genie_sim_engine/AGENTS.md#urdfxacro-authoring-as-the-source-of-truth).

**KDL constraint**

Adding `<inertial>` to the root would be the obvious fix but
**breaks KDL** (`urdf_parser`'s kinematics library, used by
`robot_state_publisher` and MoveIt): KDL emits a "Root link X has
inertial properties..." warning and refuses to compute kinematics for
downstream consumers.

**The pattern**

Add a minimum-impact `<visual>` to the URDF root: a 1mm sphere. This
flips `is_ghost_link → False` without affecting KDL (which only
forbids `<inertial>` on the root), without affecting RViz / MoveIt
appearance (1mm at the origin is invisible at any practical zoom), and
without affecting collision (it has no `<collision>` block).

```xml
<link name="base_link">
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry><sphere radius="0.001"/></geometry>
  </visual>
</link>
```

Any robot following the UR-style frame + `_inertia` sibling pattern
(empty `base_link` → fixed-joint sibling `base_link_inertia` carrying
the actual mass) needs this — that's most robots in this package.

The tradeoff: `<visual>` on the root link is technically observable via
RViz and `robot_state_publisher`'s `/tf` graph, but a 1mm sphere is
invisible at any normal viewing distance and adds no meaningful TF
graph noise.

**Robots applying this pattern**

| Robot | Root link | Reason root is a kinematic frame |
|---|---|---|
| `agilex/aloha` | `base_link` | UR-style frame + `base_link_inertia` sibling, plus mobile platform with chassis & wheels as siblings |
| `agilex/piper` | `base_link` | UR-style frame + `base_link_inertia` sibling |
| `agilex/piper_no_gripper` | `world` | Standard ROS world-anchor frame; chain starts at fixed `world → base_link` |
| `arx/acone` | `base_link` | UR-style frame + `base_link_inertia` sibling; bimanual ("AC1") |
| `arx/x5` | `base_link` | UR-style frame + `base_link_inertia` sibling |
| `franka/fr3` | `base_link` | UR-style frame + `base_link_inertia` (carries upstream `link0`) |
| `universal_robots/ur5` | `${tf_prefix}base_link` | UR-style frame, sibling `arm_base_link` for UR-internal frame, sibling `base_link_inertia` |

The `genie/g2/*` xacros aren't ghost-rooted (their root links carry
visual/collision directly), so they don't need this fix.

**How to check**

Run the diagnose tool. A clean tree should show no
`[ghost-root]` warnings:

```bash
python3 scripts/diagnose_urdf.py --dry-run robots/
```

If a new robot you're adding shows up in `[ghost-root]`, apply this
pattern in the xacro. Each affected xacro carries an inline comment
referencing this AGENTS.md section, so the design intent is greppable
from any robot file.

---

## Design references

The conventions enforced by `scripts/diagnose_urdf.py` and assumed
throughout the package come from a mix of formal ROS specs and
community/vendor practice. Cited deliberately so future edits don't
mis-attribute or over-extend any single spec.

| Convention | Source | Notes |
|---|---|---|
| Coordinate frame names — `base_link`, `base_footprint`, `odom`, `map` | [REP-105](https://www.ros.org/reps/rep-0105.html) (mobile bases) | The frames the navigation stack and TF tree expect on a mobile platform. `base_footprint` is the floor-projected anchor; `base_link` is rigidly attached to the platform body. |
| Coordinate frame names — humanoid extensions (`l_gripper`, `r_gripper`, `torso`, `gaze`, `l_sole` …) | [REP-120](https://www.ros.org/reps/rep-0120.html) (humanoids) | Extends REP-105 for humanoid robots. Note that REP-120 covers **frame naming only** — it does NOT specify how to author `<inertial>`, `<collision>`, or the kinematic frame + `_inertia` sibling pattern (those are not in any REP). |
| URDF root link must be massless | KDL parser convention; `urdfdom` / `robot_state_publisher` source. Not a REP. | If `<inertial>` is authored on the URDF root, KDL emits the "Root link X has inertial properties..." warning and refuses to compute kinematics for downstream consumers (MoveIt, ROS controllers). The diagnose tool excludes the root from auto-injection for this reason. |
| Frame + `_inertia` sibling pattern (e.g. `base_link` + `base_link_inertia`) | Universal Robots URDFs and several vendor descriptions; **not in any REP**. | The named link stays a pure kinematic frame for clean joint-axis math; a fixed-joint sibling carries the actual mass and visual mesh. The diagnose tool detects this pattern by literal `<name>_inertia` lookup and skips the frame partner. |
| URDF/SDF formal grammar | [URDF XML spec](http://wiki.ros.org/urdf/XML), [`urdfdom`](https://github.com/ros/urdfdom) | The parser ROS 2 actually uses. The diagnose tool's permissive XML pre-parse mirrors `urdfdom`'s tolerance for xacro fragments. |
| Isaac Sim 6.0 `is_ghost_link` branch | [`urdf_usd_converter._impl.link.physics_joints`](file:///opt/isaacsim/exts/isaacsim.asset.importer.urdf/pip_prebundle/urdf_usd_converter/_impl/link.py) source | Triggered when the URDF root has no `<visual>`, `<collision>`, AND `<inertial>`. Authors broken `body0` refs for every joint. The diagnose tool's "ghost-root" check catches this case before it reaches the converter. |
| Wheel-as-cylinder collider rationale | PhysX 5 contact-pair docs; engine performance characterization | Cylinder vs ground-plane collision is ~10× faster than triangle-mesh and numerically more stable for sustained contact. Not codified in any spec — empirical from the simulator side. |

When extending the diagnose tool's rule set, prefer rules that cite a
formal source (REP / URDF spec / parser source) over heuristics. When a
heuristic is the only option (e.g., wheel name detection), make it
conservative — flag for the operator rather than auto-fix — so a future
robot with non-standard naming doesn't silently get the wrong fix.

---

## Routing rules

- Universal xacro entry → `xacro/robot.xacro`
- Per-robot description files → `robots/<robot_name>/`
- OBJ mesh-name normalizer → `scripts/normalize_obj_names.py`
- URDF/xacro diagnose + fix → `scripts/diagnose_urdf.py`
- Third-party license notices → `THIRD_PARTY_NOTICES`
- URDF→USD conversion logic → `../genie_sim_engine/scripts/assemble_robot.py`
- Scene YAML robot block parsing → `../genie_sim_bringup/launch/utils.py`
