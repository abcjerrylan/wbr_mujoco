#!/usr/bin/env python3
"""Fix STL meshes for MuJoCo import.

MuJoCo requires binary STL with 1..200000 faces. This script:
  - converts ASCII STL to binary
  - decimates meshes that exceed the face limit
  - replaces empty (0-face) site markers with a tiny placeholder triangle
"""

from __future__ import annotations

import argparse
import shutil
import struct
from pathlib import Path

import numpy as np
import pymeshlab
import trimesh

MAX_FACES = 200_000
TARGET_FACES = 180_000  # leave margin below the hard limit


def read_face_count(path: Path) -> int | None:
    data = path.read_bytes()
    if data[:5].lower() == b"solid":
        mesh = trimesh.load(str(path), force="mesh")
        return len(mesh.faces)
    if len(data) < 84:
        return None
    return struct.unpack("<I", data[80:84])[0]


def write_placeholder_tetrahedron(path: Path) -> None:
    """Write a tiny binary STL tetrahedron (for empty site markers).

    MuJoCo needs at least 1 face and 4 vertices; a tetrahedron satisfies both.
    """
    scale = 1e-4
    verts = np.array(
        [
            [0.0, 0.0, 0.0],
            [scale, 0.0, 0.0],
            [0.0, scale, 0.0],
            [0.0, 0.0, scale],
        ],
        dtype=np.float64,
    )
    faces = [(0, 2, 1), (0, 1, 3), (0, 3, 2), (1, 2, 3)]

    header = b"MuJoCo placeholder" + b"\0" * (80 - 18)
    face_data = b""
    for i, j, k in faces:
        tri = verts[[i, j, k]]
        normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
        norm_len = np.linalg.norm(normal)
        if norm_len > 0:
            normal /= norm_len
        else:
            normal = np.array([0.0, 0.0, 1.0])
        face_data += struct.pack("<3f", *normal.astype(np.float32))
        for v in tri:
            face_data += struct.pack("<3f", *v.astype(np.float32))
        face_data += struct.pack("<H", 0)

    path.write_bytes(header + struct.pack("<I", len(faces)) + face_data)


def decimate_mesh(path: Path, target_faces: int) -> None:
    ms = pymeshlab.MeshSet()
    ms.load_new_mesh(str(path))
    current_faces = ms.current_mesh().face_number()
    if current_faces <= target_faces:
        ms.save_current_mesh(str(path), binary=True)
        return
    ms.meshing_decimation_quadric_edge_collapse(targetfacenum=target_faces)
    ms.save_current_mesh(str(path), binary=True)


def convert_to_binary(path: Path) -> None:
    mesh = trimesh.load(str(path), force="mesh")
    if isinstance(mesh, trimesh.Scene):
        mesh = trimesh.util.concatenate(tuple(mesh.geometry.values()))
    mesh.export(str(path))


def process_stl(path: Path, backup_dir: Path | None, dry_run: bool) -> str:
    face_count = read_face_count(path)
    if face_count is None:
        return f"skip (unreadable): {path.name}"

    action = "ok"
    if face_count == 0:
        action = "placeholder"
    elif face_count > MAX_FACES:
        action = f"decimate {face_count}->{TARGET_FACES}"
    else:
        data = path.read_bytes()
        if data[:5].lower() == b"solid":
            action = f"ascii->binary ({face_count} faces)"

    if dry_run:
        return f"{action}: {path.name}"

    if backup_dir is not None:
        backup_dir.mkdir(parents=True, exist_ok=True)
        shutil.copy2(path, backup_dir / path.name)

    if face_count == 0:
        write_placeholder_tetrahedron(path)
    elif face_count > MAX_FACES:
        decimate_mesh(path, TARGET_FACES)
    elif path.read_bytes()[:5].lower() == b"solid":
        convert_to_binary(path)
    else:
        # already valid binary; ensure trimesh can round-trip cleanly
        new_count = read_face_count(path)
        if new_count is None or new_count < 1 or new_count > MAX_FACES:
            convert_to_binary(path)

    final_faces = read_face_count(path)
    return f"{action} => {final_faces} faces: {path.name}"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--dir",
        type=Path,
        default=Path(__file__).resolve().parent,
        help="directory containing STL files (default: script directory)",
    )
    parser.add_argument(
        "--backup",
        type=Path,
        default=None,
        help="backup originals here (default: <dir>/stl_backup)",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="overwrite STL files without creating backups",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="only report what would be changed",
    )
    args = parser.parse_args()

    stl_dir = args.dir.resolve()
    backup_dir = None if args.no_backup else (args.backup or stl_dir / "stl_backup")

    stl_files = sorted(stl_dir.glob("*.STL")) + sorted(stl_dir.glob("*.stl"))
    # deduplicate case-insensitive duplicates on case-insensitive filesystems
    seen: set[str] = set()
    unique_files: list[Path] = []
    for path in stl_files:
        key = path.name.lower()
        if key not in seen:
            seen.add(key)
            unique_files.append(path)

    if not unique_files:
        print(f"No STL files found in {stl_dir}")
        return

    print(f"Processing {len(unique_files)} STL files in {stl_dir}")
    if backup_dir and not args.dry_run:
        print(f"Backups -> {backup_dir}")

    for path in unique_files:
        result = process_stl(path, backup_dir, args.dry_run)
        print(result)

    if not args.dry_run:
        print("\nVerifying with MuJoCo...")
        import mujoco

        failed = []
        for path in unique_files:
            xml = (
                f'<mujoco><asset><mesh name="m" file="{path}"/>'
                f'</asset><worldbody><geom type="mesh" mesh="m"/></worldbody></mujoco>'
            )
            try:
                mujoco.MjModel.from_xml_string(xml)
            except Exception as exc:
                failed.append((path.name, str(exc)))

        if failed:
            print("MuJoCo verification FAILED:")
            for name, err in failed:
                print(f"  {name}: {err}")
            raise SystemExit(1)
        print("All STL files load successfully in MuJoCo.")


if __name__ == "__main__":
    main()
