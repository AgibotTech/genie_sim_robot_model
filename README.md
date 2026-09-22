# genie_sim_robot_model

Robot descriptions for GenieSim — xacro / URDF sources, meshes (`.dae`,
`.STL`, `.obj`), pre-baked USDs, and the mesh-prep tools that keep
those assets clean enough for Isaac Sim's URDF→USD importer.

## Robots shipped

| Tier | Robots | What it means |
|---|---|---|
| **Tier 1 — actively supported** | Genie G2 (`crs` / `crsB` arm × `omnipicker` / `swiftpicker` gripper) | Continuously validated against every release. Physics tuning, contact compliance, mimic constraints, mobile-base pin/free behaviour are all maintained. Bug reports get triaged here first. |
| **Tier 2 — reference only** | `agilex/aloha`, `agilex/piper`, `arx/x5`, `arx/acone`, `franka/fr3`, `universal_robots/ur5` | URDF/xacro kept correct; URDF→USD import expected to work. Scene yamls and physics tuning may be stale — treat as starting points. |

Bring-your-own robots are supported: drop a new xacro tree under
`robots/<vendor>/<model>/`, generate a flat URDF, and the engine's
assemble pipeline takes it from there.

## Layout

```
robots/                               # per-robot xacro / URDF sources + meshes
├── genie/g2/                         # Tier 1
├── agilex/{aloha,piper}/             # Tier 2
├── arx/{x5,acone}/                   # Tier 2
├── franka/fr3/                       # Tier 2
└── universal_robots/ur5/             # Tier 2
urdf/                                 # generated flat URDFs (consumed by MoveIt / sim)
scripts/                              # mesh-prep / diagnostic tools
└── diagnose_urdf.py, normalize_obj_names.py, fix_dae_units.py,
    recompute_inertia.py, copy_dae_material.py, …
```

## Mesh-prep tools

DCC exports rarely arrive in a state the Isaac Sim importer is happy
with. The `scripts/` tools fix the common issues:

- **`diagnose_urdf.py`** — finds links missing `<inertial>` blocks
  (the Isaac Sim 6.0 converter's `is_ghost_link` branch trips on these
  and breaks the articulation). Has a `--fix` mode that injects a
  placeholder.
- **`normalize_obj_names.py`** — rewrites every `.obj`'s `o <name>`
  directive to match the file stem, eliminating cross-file name
  collisions the Isaac Sim 6.0 converter de-dupes against.
- **`fix_dae_units.py`** — scans `.dae` files exported in millimetres
  and rescales them to metres (matches the URDF / SDF / Newton / PhysX
  convention).
- **`copy_dae_material.py`** — copies the COLLADA material plumbing
  from a donor DAE into a target DAE (e.g. pick up the crsB look on a
  crs arm mesh exported without materials).
- **`recompute_inertia.py`** — recomputes inertial tensors from
  meshes under uniform density.

Run these once after every batch of DCC exports.

## When you'd touch this package

- Adding a new robot — drop xacros under `robots/<vendor>/<model>/`.
- Fixing meshes that don't import cleanly into Isaac Sim — start with
  the diagnostic tools in `scripts/`.
- Tuning a robot's URDF (joint limits, mass, inertia, mimic
  constraints).
- Adding a new gripper / end-effector variant — extend the existing
  G2 xacro composition pattern (`g2_<arm>_<gripper>.urdf.xacro`).

## Mechanics

See [AGENTS.md](AGENTS.md) for the robot composition pattern, mesh
conventions per format (`.dae` / `.obj` / `.STL`), the Isaac Sim
URDF→USD path each importer takes, and the gotchas the tools fix.

## License

AgiBot-authored content is released under the
[Apache License 2.0](LICENSE). That covers the xacro dispatch layer
(`xacro/`), the Python helpers (`genie_sim_robot_model/`, `scripts/`),
the launch and rviz configs, and the Genie G2 robot description under
`robots/genie/g2/`.

Bundled third-party robot descriptions each retain their upstream
license:

| Robot | Directory | License |
|---|---|---|
| Franka FR3 | `robots/franka/fr3/` | Apache-2.0 |
| Universal Robots UR5 + Robotiq | `robots/universal_robots/ur5/` | BSD-3-Clause; meshes under UR graphical-documentation T&Cs |
| Agilex Piper | `robots/agilex/piper/` | MIT |
| Agilex Aloha | `robots/agilex/aloha/` | no license declared upstream — contact AgileX before commercial use |
| ARX X5 / Acone | `robots/arx/` | BSD |
| Wuji hand | `robots/genie/g2/meshes/gripper/wuji/` + wuji xacro macros | MIT (© 2025 Wuji Technology) |

See [THIRD_PARTY_NOTICES](THIRD_PARTY_NOTICES) for the full per-robot
terms and attribution requirements. Note that the Wuji hand assets sit
inside the otherwise Apache-2.0 `robots/genie/g2/` tree but remain under
Wuji's MIT license.
