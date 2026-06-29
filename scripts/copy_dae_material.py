#!/usr/bin/env python3
# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""Copy a COLLADA (.dae) material from a donor file into a target file.

What this exists for
--------------------

Some hand-authored or stock DAEs ship with a no-op ``DefaultMaterial``
(white diffuse, no styling, no images) instead of the per-link colour
the rest of the robot uses.  The Genie ``g2/crs`` arm meshes are the
canonical case: each ``arm_linkN.dae`` exports under ``DefaultMaterial``,
while ``crsB``'s ``arm_l_linkN.dae`` ships with the real material
(``black`` for links 1/6/7, ``white`` for links 2-5) and renders the way
operators expect.

Rather than re-export the meshes through a DCC, this script copies the
material plumbing from a donor DAE into a target DAE in place.  The
geometry (``<source>`` / ``<vertices>`` / primitive indices) is left
strictly untouched.

What is copied
--------------

For each (target, donor) pair the script:

  1. Replaces ``<library_materials>`` and ``<library_effects>`` in the
     target with the donor's blocks verbatim.
  2. Replaces ``<library_images>`` in the target with the donor's block
     (or removes the empty one if the donor has none).
  3. Rewrites every ``<instance_material symbol="…" target="#…">`` in
     the target so symbol / target match the donor's material binding.
  4. Rewrites every primitive element's ``material="…"`` attribute
     (``<triangles>`` / ``<polylist>`` / ``<polygons>`` / ``<lines>`` /
     ``<tristrips>`` / ``<trifans>``) so it points at the donor symbol.

The target must contain exactly one material and exactly one bound
symbol; the donor must too.  Multi-material DAEs are rejected with a
clear error — they need a different tool, not this one.

Usage
-----

Single pair::

    scripts/copy_dae_material.py \\
      target.dae donor.dae

Batch — copy ``crsB`` left-side materials into the ``crs`` arm::

    for n in 1 2 3 4 5 6 7; do
      scripts/copy_dae_material.py \\
        robots/genie/g2/meshes/arm/crs/arm_link${n}.dae \\
        robots/genie/g2/meshes/arm/crsB/arm_l_link${n}.dae
    done

The script is idempotent: running it twice in a row produces the same
file the second time.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Tuple

# Primitive elements whose ``material="…"`` attribute participates in
# material binding.  Lines / linestrips technically can't carry material
# the same way but COLLADA accepts the attribute, so we rewrite it for
# consistency if present.
_PRIMITIVE_RE = re.compile(
    r'(<(?:triangles|polylist|polygons|lines|linestrips|tristrips|trifans)\b[^>]*\bmaterial=")([^"]*)(")'
)

_INSTANCE_MATERIAL_RE = re.compile(r'(<instance_material\b[^>]*\bsymbol=")([^"]*)("\s*[^>]*\btarget="#)([^"]*)(")')

_LIBRARY_MATERIALS_RE = re.compile(r"<library_materials\b.*?</library_materials>", re.DOTALL)
_LIBRARY_EFFECTS_RE = re.compile(r"<library_effects\b.*?</library_effects>", re.DOTALL)
_LIBRARY_IMAGES_FULL_RE = re.compile(r"<library_images\b.*?</library_images>", re.DOTALL)
_LIBRARY_IMAGES_SHORT_RE = re.compile(r"<library_images\s*/>")

_MATERIAL_ID_RE = re.compile(r'<material\s+id="([^"]+)"')


def _block(pattern: re.Pattern, text: str, label: str, path: Path) -> str:
    m = pattern.search(text)
    if not m:
        raise SystemExit(f"❌ {path}: missing required <{label}> block")
    return m.group(0)


def _binding(text: str, path: Path) -> Tuple[str, str]:
    """Return (symbol, target) from the file's first instance_material binding."""
    m = _INSTANCE_MATERIAL_RE.search(text)
    if not m:
        raise SystemExit(f"❌ {path}: no <instance_material symbol=… target=#…> found")
    return m.group(2), m.group(4)


def _check_single_material(text: str, path: Path) -> None:
    ids = _MATERIAL_ID_RE.findall(text)
    if len(ids) != 1:
        raise SystemExit(
            f"❌ {path}: expected exactly 1 <material id=…>, found {len(ids)}. "
            "This script only handles single-material DAEs; use a DCC re-export "
            "for multi-material assets."
        )


def copy_material(target_path: Path, donor_path: Path) -> bool:
    """Copy donor's material plumbing into target.  Returns True if changed."""
    target = target_path.read_text(encoding="utf-8")
    donor = donor_path.read_text(encoding="utf-8")

    _check_single_material(target, target_path)
    _check_single_material(donor, donor_path)

    donor_materials = _block(_LIBRARY_MATERIALS_RE, donor, "library_materials", donor_path)
    donor_effects = _block(_LIBRARY_EFFECTS_RE, donor, "library_effects", donor_path)
    # Images may be absent on the donor — that's fine; we still want to
    # propagate "no images" to the target if the donor has none.
    donor_images_match = _LIBRARY_IMAGES_FULL_RE.search(donor) or _LIBRARY_IMAGES_SHORT_RE.search(donor)
    donor_images = donor_images_match.group(0) if donor_images_match else None

    donor_symbol, donor_target = _binding(donor, donor_path)

    new = target
    new = _LIBRARY_MATERIALS_RE.sub(lambda _m: donor_materials, new, count=1)
    new = _LIBRARY_EFFECTS_RE.sub(lambda _m: donor_effects, new, count=1)

    target_has_images = bool(_LIBRARY_IMAGES_FULL_RE.search(new) or _LIBRARY_IMAGES_SHORT_RE.search(new))
    if donor_images is not None:
        if target_has_images:
            new = _LIBRARY_IMAGES_FULL_RE.sub(lambda _m: donor_images, new, count=1)
            new = _LIBRARY_IMAGES_SHORT_RE.sub(lambda _m: donor_images, new, count=1)
        else:
            # No images block in target — splice donor's images right
            # before <library_effects>.  Spec order is asset → images →
            # effects → materials, so this keeps the file canonical.
            new = new.replace(donor_effects, donor_images + "\n  " + donor_effects, 1)
    elif target_has_images:
        # Donor has no images; drop the target's empty (or stale) block
        # so the two files end up structurally aligned.
        new = _LIBRARY_IMAGES_FULL_RE.sub("", new, count=1)
        new = _LIBRARY_IMAGES_SHORT_RE.sub("", new, count=1)

    # Rewrite every <instance_material symbol="…" target="#…"> binding.
    new = _INSTANCE_MATERIAL_RE.sub(
        lambda m: f"{m.group(1)}{donor_symbol}{m.group(3)}{donor_target}{m.group(5)}",
        new,
    )
    # Rewrite every primitive element's material="…" attribute.
    new = _PRIMITIVE_RE.sub(lambda m: f"{m.group(1)}{donor_symbol}{m.group(3)}", new)

    if new == target:
        return False
    target_path.write_text(new, encoding="utf-8")
    return True


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("target", type=Path, help="DAE to modify in place")
    parser.add_argument("donor", type=Path, help="DAE to copy material plumbing from")
    args = parser.parse_args(argv)

    for p in (args.target, args.donor):
        if not p.is_file():
            print(f"❌ Not a file: {p}", file=sys.stderr)
            return 2

    changed = copy_material(args.target, args.donor)
    if changed:
        print(f"✅ {args.target} ← material from {args.donor}")
    else:
        print(f"(unchanged) {args.target} already matches {args.donor}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
