#!/usr/bin/env python3
"""Put the parts of the x500 quad together into one binary .stl file.

The x500 is the 500 mm quad of the PX4 simulator, in the PX4-gazebo-models
repository on GitHub (BSD 3-Clause; see LICENSE-x500 here). Its model is
spread over a description, model.sdf, and five mesh files: the frame and two
motor halves as Collada (.dae), and two propellers as .stl. The view
program reads none of those, so this script places every part as model.sdf
says and writes the triangles into one file.

Usage:
    ./make_x500.py DIRECTORY x500.stl

DIRECTORY holds model.sdf and the five mesh files, as downloaded from
models/x500_base/ and models/x500_base/meshes/ of the repository.

The file written is in metres, with x forward, y left and z up, as the
simulator has it. Each triangle carries its colour in the two spare bytes of
the format: bit 15 set, then five bits each of red, green and blue. Only the
Python standard library is needed.
"""

import math
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

# The colour of a part whose file gives none: the propellers.
PROPELLER = (0.10, 0.10, 0.10)


# A placement is a 3×4 table: a 3×3 turn and stretch, and a move in the last
# column. Points are (x, y, z).

IDENTITY = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0]]


def combine(outer, inner):
    """The placement that does `inner` first and `outer` after it."""
    rows = []
    for r in range(3):
        row = [sum(outer[r][k] * inner[k][c] for k in range(3)) for c in range(4)]
        row[3] += outer[r][3]
        rows.append(row)
    return rows


def place(m, p):
    return tuple(m[r][0] * p[0] + m[r][1] * p[1] + m[r][2] * p[2] + m[r][3] for r in range(3))


def from_pose(text):
    """An SDF pose, `x y z roll pitch yaw`: turn about x, then y, then z, then move."""
    x, y, z, roll, pitch, yaw = (float(w) for w in (text or "0 0 0 0 0 0").split())
    sr, cr = math.sin(roll), math.cos(roll)
    sp, cp = math.sin(pitch), math.cos(pitch)
    sy, cy = math.sin(yaw), math.cos(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, x],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, y],
        [-sp, cp * sr, cp * cr, z],
    ]


def from_scale(text):
    sx, sy, sz = (float(w) for w in (text or "1 1 1").split())
    return [[sx, 0, 0, 0], [0, sy, 0, 0], [0, 0, sz, 0]]


def strip_namespaces(root):
    for element in root.iter():
        element.tag = element.tag.split("}")[-1]
    return root


def read_dae(path):
    """The triangles of a Collada file, as (three points, colour)."""
    root = strip_namespaces(ET.parse(path).getroot())

    def by_id(tag):
        return {e.get("id"): e for e in root.iter(tag)}

    geometries, materials, effects = by_id("geometry"), by_id("material"), by_id("effect")

    def colour_of(material_id):
        material = materials.get(material_id)
        if material is None:
            return PROPELLER
        effect = effects.get(material.find("instance_effect").get("url")[1:])
        diffuse = effect.find(".//diffuse/color")
        if diffuse is None:
            return (0.5, 0.5, 0.5)
        return tuple(float(w) for w in diffuse.text.split()[:3])

    def geometry_triangles(geometry, colours):
        mesh = geometry.find("mesh")
        sources = {}
        for source in mesh.findall("source"):
            numbers = [float(w) for w in source.find("float_array").text.split()]
            sources[source.get("id")] = [tuple(numbers[i : i + 3]) for i in range(0, len(numbers), 3)]
        positions = {
            v.get("id"): sources[v.find("input[@semantic='POSITION']").get("source")[1:]]
            for v in mesh.findall("vertices")
        }
        found = []
        for group in mesh.findall("triangles"):
            inputs = group.findall("input")
            step = max(int(i.get("offset")) for i in inputs) + 1
            vertex = group.find("input[@semantic='VERTEX']")
            points = positions[vertex.get("source")[1:]]
            offset = int(vertex.get("offset"))
            index = [int(w) for w in group.find("p").text.split()][offset::step]
            colour = colours.get(group.get("material"), (0.5, 0.5, 0.5))
            for i in range(0, len(index) - 2, 3):
                found.append((tuple(points[k] for k in index[i : i + 3]), colour))
        return found

    triangles = []

    def walk(node, above):
        here = above
        for matrix in node.findall("matrix"):
            n = [float(w) for w in matrix.text.split()]
            here = combine(here, [n[0:4], n[4:8], n[8:12]])
        for instance in node.findall("instance_geometry"):
            colours = {
                bind.get("symbol"): colour_of(bind.get("target")[1:])
                for bind in instance.iter("instance_material")
            }
            for points, colour in geometry_triangles(geometries[instance.get("url")[1:]], colours):
                triangles.append((tuple(place(here, p) for p in points), colour))
        for child in node.findall("node"):
            walk(child, here)

    for scene in root.iter("visual_scene"):
        for node in scene.findall("node"):
            walk(node, IDENTITY)
    return triangles


def read_stl(path):
    """The triangles of an .stl file, binary or text, in the propeller's colour."""
    data = open(path, "rb").read()
    # A binary file is 84 bytes and then 50 for each triangle. Its heading
    # may begin with `solid`, as a text file does, so the length decides.
    if len(data) >= 84 and len(data) == 84 + 50 * struct.unpack("<I", data[80:84])[0]:
        found = []
        for at in range(84, len(data), 50):
            n = struct.unpack("<9f", data[at + 12 : at + 48])
            found.append(((n[0:3], n[3:6], n[6:9]), PROPELLER))
        return found
    points = []
    for line in data.decode(errors="replace").splitlines():
        words = line.split()
        if words and words[0] == "vertex":
            points.append(tuple(float(w) for w in words[1:4]))
    return [(tuple(points[i : i + 3]), PROPELLER) for i in range(0, len(points) - 2, 3)]


def main():
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    directory, out = Path(sys.argv[1]), sys.argv[2]

    meshes = {}
    triangles = []
    model = ET.parse(directory / "model.sdf").getroot().find("model")
    for link in model.findall("link"):
        link_place = from_pose(link.findtext("pose"))
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                continue  # a flat picture of a label; left out
            name = mesh.findtext("uri").split("/")[-1]
            if name not in meshes:
                reader = read_dae if name.endswith(".dae") else read_stl
                meshes[name] = reader(directory / name)
            placement = combine(
                link_place,
                combine(from_pose(visual.findtext("pose")), from_scale(mesh.findtext("scale"))),
            )
            for points, colour in meshes[name]:
                triangles.append((tuple(place(placement, p) for p in points), colour))
            print(f"{visual.get('name'):28s} {name:20s} {len(meshes[name]):7d} triangles", file=sys.stderr)

    with open(out, "wb") as f:
        f.write(b"x500 quad, PX4-gazebo-models, BSD 3-Clause; metres, z up".ljust(80, b" "))
        f.write(struct.pack("<I", len(triangles)))
        for points, colour in triangles:
            red, green, blue = (min(31, max(0, round(part * 31))) for part in colour)
            spare = 0x8000 | red << 10 | green << 5 | blue
            f.write(struct.pack("<12fH", 0.0, 0.0, 0.0, *points[0], *points[1], *points[2], spare))

    low = [min(p[a] for t, _ in triangles for p in t) for a in range(3)]
    high = [max(p[a] for t, _ in triangles for p in t) for a in range(3)]
    print(f"{out}: {len(triangles)} triangles", file=sys.stderr)
    print("from", [round(v, 3) for v in low], "to", [round(v, 3) for v in high], "m", file=sys.stderr)


if __name__ == "__main__":
    main()
