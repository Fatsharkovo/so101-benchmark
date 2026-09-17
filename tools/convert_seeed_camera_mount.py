"""Reproduce the Seeed wrist mesh from the pinned STEP file (optional gmsh dependency).

Run: uv run --no-project --with gmsh==4.15.2 --with numpy python
     tools/convert_seeed_camera_mount.py /path/to/soarm_soft_gripper.stp output.stl
"""

from __future__ import annotations

import argparse
import hashlib
import json
import struct
from pathlib import Path

import gmsh
import numpy as np


def convert(source: Path, destination: Path) -> None:
    assets = Path(__file__).resolve().parents[1] / "src/lerobot_env_so101/assets/so101"
    metadata = json.loads((assets / "seeed_camera_mount_manifest.json").read_text())
    if hashlib.sha256(source.read_bytes()).hexdigest() != metadata["source_sha256"]:
        raise ValueError("STEP does not match the pinned source revision")
    rotation = np.array(metadata["alignment"]["rotation"])
    translation = np.array(metadata["alignment"]["translation_mm"])
    gmsh.initialize()
    try:
        gmsh.option.setNumber("General.Terminal", 0)
        gmsh.model.occ.importShapes(str(source))
        gmsh.model.occ.synchronize()
        volumes = gmsh.model.getEntities(3)
        selected = [
            item for item in volumes if gmsh.model.getEntityName(*item).endswith("/SO-ARM101_CAMERA_MOUNT")
        ]
        if len(selected) != 1:
            raise ValueError("Expected one SO-ARM101_CAMERA_MOUNT solid")
        gmsh.model.occ.remove([item for item in volumes if item not in selected], recursive=True)
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.MeshSizeMin", 0.7)
        gmsh.option.setNumber("Mesh.MeshSizeMax", 1.2)
        gmsh.model.mesh.generate(2)
        tags, coordinates, _ = gmsh.model.mesh.getNodes()
        lookup = {int(tag): index for index, tag in enumerate(tags)}
        vertices = (np.array(coordinates).reshape(-1, 3) @ rotation.T + translation) / 1000
        kinds, _, element_nodes = gmsh.model.mesh.getElements(2)
        triangles = np.concatenate(
            [
                vertices[np.array([lookup[int(node)] for node in nodes]).reshape(-1, 3)]
                for kind, nodes in zip(kinds, element_nodes, strict=True)
                if kind == 2
            ]
        )
        with destination.open("wb") as stream:
            header = b"Seeed wiki-linked SO-ARM101_CAMERA_MOUNT; gripper coordinates, metres"
            stream.write(header.ljust(80, b"\0"))
            stream.write(struct.pack("<I", len(triangles)))
            for triangle in triangles:
                normal = np.cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
                normal /= max(np.linalg.norm(normal), 1e-30)
                stream.write(struct.pack("<12fH", *normal, *triangle.ravel(), 0))
        print(f"Exported {len(triangles)} triangles to {destination}")
    finally:
        gmsh.finalize()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    args = parser.parse_args()
    convert(args.source, args.destination)
