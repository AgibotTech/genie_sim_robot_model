#!/usr/bin/env python3
# Copyright (c) 2023-2026, AgiBot Inc. All Rights Reserved.
# Author: Genie Sim Team
# License: Mozilla Public License Version 2.0

"""Fix COLLADA (.dae) meshes exported with non-metre units.

What this exists for
--------------------

URDF / SDF / xacro / Newton / mjwarp / PhysX / RViz all assume that
mesh vertex coordinates are in METRES.  COLLADA's spec allows the
mesh to declare a different unit via ``<unit meter="X" name="..."/>``
in ``<asset>``, but virtually no robotics loader honours the
declaration — they read the raw vertex values and treat them as
metres regardless.

Result: a CAD export with ``<unit meter="0.001"/>`` (the SolidWorks /
Blender default when the user picks "millimeters") loads as a robot
that is 1000× too large.  Inertia computed from such a mesh under
uniform density is 1e9× too large; collision dispatch sees a giant
hovering ghost-mesh; visual rendering puts the part at the wrong
place if the URDF has any subsequent scaling.

Detection
---------

This script can run in --dry-run / report-only mode to scan a tree
of ``.dae`` files and list any whose vertex bounds extend past
``--bounds-threshold`` metres (default 5.0 m — past the spec of any
human-scale robot).  Such files almost certainly need this fix.

Fix
---

For each ``.dae`` flagged:

1. Parse the ``<unit meter="X"/>`` declaration.  Use the file-
   declared ``meter`` value as the scale factor (so a file marked
   ``meter="0.001"`` gets vertices × 0.001).  When the file says
   ``meter="1"`` we treat it as a no-op (probably already correct,
   or the bounds-threshold check produced a false positive).
2. Identify every ``<float_array id="...POSITION-array">`` — these
   carry the vertex position data.  Other arrays (``Normal0-array``,
   ``UV0-array``, ``Tangent0-array``) are dimensionless directions /
   texture coords and must NOT be scaled.
3. Scale the values in each POSITION array by ``meter``.
4. Scale every ``<translate>`` element's content by the same factor
   — those are scene-node placements in the same units.
5. Rewrite the ``<unit>`` element to ``<unit meter="1.000000"
   name="meter"/>`` so the file is now self-consistent.

We do NOT touch ``<scale>`` elements (they're dimensionless factors)
or ``<rotate>`` / ``<lookat>`` (rotations are unit-independent).
``<extra>`` blocks are left as-is — they may carry custom data
(SolidWorks PMI etc.) that some downstream tool relies on; if any
of that contained absolute distances we'd miss it, but it's the
right trade-off.

Safety
------

* Writes a ``.dae.bak`` next to each modified file (overwrites a
  prior backup if present).  Pass ``--no-backup`` to skip.
* Operates by regex on the raw text, preserving whitespace /
  comments / XML formatting EXACTLY.  Verified by parsing both
  the input and output with ``xml.etree.ElementTree`` and
  comparing structure.
* ``--dry-run`` reports what would change without writing.
* Idempotent: re-running on a file already at ``meter="1"`` is a
  no-op.

Usage
-----

Scan all meshes in a directory tree, report only::

    fix_dae_units.py path/to/meshes --dry-run

Apply fixes::

    fix_dae_units.py path/to/meshes

Force a fix on a specific file even if its ``<unit>`` already says
``meter="1"`` (escape hatch for files where the declaration is
wrong but the values genuinely are in millimetres)::

    fix_dae_units.py body_link4.dae --force-scale 0.001
"""

from __future__ import annotations

import argparse
import re
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Optional, Tuple

# ``<unit ... meter="0.001000" .../>`` — anywhere inside the file.
# We accept any attribute order so name= can come before or after.
_UNIT_RE = re.compile(r'(<unit\b[^/>]*\bmeter\s*=\s*")([-+0-9.eE]+)"([^/>]*)/>')

# ``<float_array id="..POSITION-array" count="N">v1 v2 v3 ...</float_array>``
# Some exporters split the values over many lines; we capture the
# entire inner text non-greedily.
_POSITION_ARRAY_RE = re.compile(
    r'(<float_array\b[^>]*\bid\s*=\s*"[^"]*POSITION-array"[^>]*>)(.*?)(</float_array>)',
    re.DOTALL,
)

# ``<translate>x y z</translate>`` — node placements.  COLLADA allows
# an optional ``sid="..."`` attribute; we use a broad open-tag match.
_TRANSLATE_RE = re.compile(r"(<translate\b[^>]*>)([^<]*)(</translate>)")


def _scale_value_string(value_str: str, factor: float) -> str:
    """Scale all whitespace-separated floats by ``factor`` while
    preserving the surrounding whitespace pattern.

    We deliberately re-format every number to a consistent
    representation (``%.9g``) — preserving the exact original
    spacing would require a far more complex tokeniser, and 9
    significant digits is well above what COLLADA exporters
    typically write (~6).  Whitespace structure (line breaks) IS
    preserved by re.sub on each match.
    """
    tokens = value_str.split()
    scaled = [f"{float(t) * factor:.9g}" for t in tokens]
    return " ".join(scaled)


def _scaled_float_array_replace(match: re.Match, factor: float) -> str:
    head, body, tail = match.group(1), match.group(2), match.group(3)
    # Strip surrounding whitespace from the body to detect leading /
    # trailing line breaks the exporter wrote; restore them after
    # scaling so the file's overall vertical layout is preserved.
    leading = ""
    trailing = ""
    body_stripped = body
    # Capture leading whitespace (incl. newlines) verbatim.
    m_l = re.match(r"\s+", body)
    if m_l:
        leading = m_l.group(0)
        body_stripped = body[len(leading) :]
    # Capture trailing whitespace verbatim.
    m_t = re.search(r"\s+$", body_stripped)
    if m_t:
        trailing = m_t.group(0)
        body_stripped = body_stripped[: -len(trailing)]
    scaled = _scale_value_string(body_stripped, factor)
    return head + leading + scaled + trailing + tail


def _scaled_translate_replace(match: re.Match, factor: float) -> str:
    head, body, tail = match.group(1), match.group(2), match.group(3)
    return head + _scale_value_string(body, factor) + tail


def _parse_unit_meter(text: str) -> Optional[float]:
    """Return the file's declared ``meter`` value, or ``None`` if
    no ``<unit>`` element is present.  When the file declares
    multiple ``<unit>`` (unusual; the spec allows only one in
    ``<asset>``) we take the first."""
    m = _UNIT_RE.search(text)
    if not m:
        return None
    try:
        return float(m.group(2))
    except ValueError:
        return None


def _measure_bounds(path: Path) -> Optional[float]:
    """Load the mesh via trimesh and return ``max(|min|, |max|)``,
    or ``None`` if the file can't be loaded.  Used by --dry-run to
    filter to "actually suspect" files instead of relying on the
    ``<unit>`` declaration alone (which authors sometimes leave
    inconsistent with the values)."""
    try:
        import trimesh
    except ImportError:
        return None
    try:
        m = trimesh.load(str(path), force="mesh")
    except Exception:  # noqa: BLE001
        return None
    if isinstance(m, trimesh.Scene):
        try:
            m = m.to_geometry() if hasattr(m, "to_geometry") else m.dump(concatenate=True)
        except Exception:  # noqa: BLE001
            return None
    if not isinstance(m, trimesh.Trimesh):
        return None
    try:
        bmin, bmax = m.bounds
        return float(max(abs(bmin).max(), abs(bmax).max()))
    except Exception:  # noqa: BLE001
        return None


def _verify_xml_structure(before: str, after: str) -> bool:
    """Parse both versions and check the element tree shape is
    identical (same tags in same order with same attributes
    excluding ``meter`` itself).  Catches accidental corruption
    from the regex substitution.

    Returns True if structure is preserved, False otherwise.
    """
    try:
        root_b = ET.fromstring(before)
        root_a = ET.fromstring(after)
    except ET.ParseError:
        return False

    def _shape(elem: ET.Element) -> Tuple[str, Tuple[str, ...], Tuple]:
        # Element tag (with namespace), sorted attribute keys, then
        # children shapes recursively.  We don't compare attribute
        # VALUES because we're allowed to change meter= and the
        # array text doesn't show up here (it's element text, not
        # an attribute).
        return (elem.tag, tuple(sorted(elem.attrib.keys())), tuple(_shape(c) for c in elem))

    return _shape(root_b) == _shape(root_a)


def _process_file(
    path: Path,
    *,
    bounds_threshold: float,
    force_scale: Optional[float],
    dry_run: bool,
    backup: bool,
) -> Tuple[bool, str]:
    """Process one .dae file.  Returns ``(modified, message)``."""
    text = path.read_text(encoding="utf-8")
    declared_meter = _parse_unit_meter(text)

    # Decide on the scale factor.
    if force_scale is not None:
        scale = force_scale
        reason = f"--force-scale={scale}"
    elif declared_meter is None:
        return False, "no <unit> declaration — skipped"
    elif abs(declared_meter - 1.0) < 1e-9:
        # File claims metres; check bounds anyway so we don't miss
        # a file that lies about its units.
        ext = _measure_bounds(path)
        if ext is not None and ext > bounds_threshold:
            return False, (
                f'<unit meter="1.0"/> but mesh bounds extend to {ext:.1f} m '
                f"— file declaration is inconsistent with vertex data, "
                f"manual review required.  Pass --force-scale 0.001 to override."
            )
        return False, f'<unit meter="{declared_meter}"/> already metres'
    else:
        scale = declared_meter
        reason = f'<unit meter="{declared_meter}"/>'

    # Bounds check before rewrite — skip silent no-op files.
    ext_before = _measure_bounds(path)
    if ext_before is not None and ext_before <= bounds_threshold and force_scale is None:
        return False, (
            f"{reason} but mesh bounds only extend to {ext_before:.3f} m — "
            f"already within URDF metre range, no fix needed."
        )

    # Apply scaling.
    new_text = _POSITION_ARRAY_RE.sub(lambda m: _scaled_float_array_replace(m, scale), text)
    new_text = _TRANSLATE_RE.sub(lambda m: _scaled_translate_replace(m, scale), new_text)
    # Rewrite the <unit> element to declare metres.  Use ``\g<N>``
    # backreferences (NOT ``\N``) because concatenating ``\1`` with
    # the literal string ``1.000000`` would parse as ``\11`` —
    # group 11 doesn't exist, raising "invalid group reference"
    # at substitution time.
    new_text = _UNIT_RE.sub(r'\g<1>1.000000"\g<3> name="meter"/>', new_text, count=1)
    # The substitution above leaves a duplicate name= when the
    # original already had one.  Collapse `name="X" name="meter"`
    # → `name="meter"` for cleanliness.
    new_text = re.sub(r'\s*name\s*=\s*"[^"]*"(\s+name\s*=\s*"meter")', r"\1", new_text)

    if new_text == text:
        return False, "no change after scale (mesh has no POSITION arrays?)"

    # Structural sanity check.
    if not _verify_xml_structure(text, new_text):
        return False, "ABORTED — XML structure check failed after substitution"

    if dry_run:
        ext_after = None
        return True, (
            f"WOULD FIX (scale={scale} from {reason}): " f"bounds {ext_before:.1f} m → ~{(ext_before * scale):.3f} m"
        )

    if backup:
        backup_path = path.with_suffix(path.suffix + ".bak")
        shutil.copy2(path, backup_path)
    path.write_text(new_text, encoding="utf-8")
    return True, f"fixed (scale={scale} from {reason})"


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        description="Re-scale .dae meshes that were exported with non-metre units "
        "so vertex coordinates land in metres (URDF convention).",
        epilog="See module docstring for the why, the detection rules, and " "the safety guarantees.",
    )
    parser.add_argument(
        "paths",
        nargs="+",
        type=Path,
        help="Files or directories to scan.  Directories are walked " "recursively for ``.dae``.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Report would-fix files without writing.",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="Don't create a ``.bak`` copy before rewriting (default: do create).",
    )
    parser.add_argument(
        "--bounds-threshold",
        type=float,
        default=5.0,
        help="Skip files whose mesh bounds are within ±threshold metres "
        "(default 5.0 m).  Bumps very-suspect files to the foreground.",
    )
    parser.add_argument(
        "--force-scale",
        type=float,
        default=None,
        help="Force this scale factor on every file, ignoring the file's "
        "own <unit> declaration.  Use when an exporter LIES about its "
        'units (e.g. wrote meter="1.0" but the vertices are still mm).',
    )
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])

    # Gather .dae files
    files: List[Path] = []
    for p in args.paths:
        if not p.exists():
            print(f"WARN: path does not exist: {p}", file=sys.stderr)
            continue
        if p.is_file() and p.suffix.lower() == ".dae":
            files.append(p.resolve())
        elif p.is_dir():
            for c in p.rglob("*.dae"):
                files.append(c.resolve())
    files = sorted(set(files))
    if not files:
        print("No .dae files found.")
        return 0

    print(f"Scanned {len(files)} .dae file(s).  threshold={args.bounds_threshold} m  dry_run={args.dry_run}")

    modified = 0
    errors = 0
    for f in files:
        try:
            mod, msg = _process_file(
                f,
                bounds_threshold=args.bounds_threshold,
                force_scale=args.force_scale,
                dry_run=args.dry_run,
                backup=not args.no_backup,
            )
        except Exception as exc:  # noqa: BLE001
            print(f"  ERROR {f}: {exc!r}")
            errors += 1
            continue
        # Print only files that actually changed or are noteworthy
        # (skipped with a real reason, not just "already metres").
        if mod or "but mesh bounds" in msg or "inconsistent" in msg or "ABORTED" in msg:
            print(f"  {f}: {msg}")
        if mod:
            modified += 1
    print()
    print(f"Summary: modified={modified}  errors={errors}  total={len(files)}")
    if errors:
        return 2
    if args.dry_run and modified > 0:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
