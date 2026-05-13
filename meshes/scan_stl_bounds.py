#!/usr/bin/env python3
import os
import struct
import glob
import math

def read_stl_vertices(path):
    with open(path, "rb") as f:
        data = f.read()

    # Try binary STL first
    if len(data) >= 84:
        tri_count = struct.unpack("<I", data[80:84])[0]
        expected_size = 84 + tri_count * 50

        if expected_size == len(data):
            verts = []
            offset = 84
            for _ in range(tri_count):
                # normal = 12 bytes, then 3 vertices = 36 bytes, attr = 2 bytes
                offset += 12
                for _ in range(3):
                    x, y, z = struct.unpack("<fff", data[offset:offset+12])
                    verts.append((x, y, z))
                    offset += 12
                offset += 2
            return verts

    # Fallback ASCII STL
    text = data.decode(errors="ignore")
    verts = []
    for line in text.splitlines():
        line = line.strip()
        if line.startswith("vertex"):
            parts = line.split()
            if len(parts) == 4:
                verts.append(tuple(float(v) for v in parts[1:4]))
    return verts

def summarize(path):
    verts = read_stl_vertices(path)
    if not verts:
        print(f"{os.path.basename(path)}: NO VERTICES FOUND")
        return

    xs = [v[0] for v in verts]
    ys = [v[1] for v in verts]
    zs = [v[2] for v in verts]

    mn = (min(xs), min(ys), min(zs))
    mx = (max(xs), max(ys), max(zs))
    center = ((mn[0]+mx[0])/2, (mn[1]+mx[1])/2, (mn[2]+mx[2])/2)
    size = (mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2])

    print(f"\n{os.path.basename(path)}")
    print(f"  min mm:    x={mn[0]:9.3f}, y={mn[1]:9.3f}, z={mn[2]:9.3f}")
    print(f"  max mm:    x={mx[0]:9.3f}, y={mx[1]:9.3f}, z={mx[2]:9.3f}")
    print(f"  center mm: x={center[0]:9.3f}, y={center[1]:9.3f}, z={center[2]:9.3f}")
    print(f"  size mm:   x={size[0]:9.3f}, y={size[1]:9.3f}, z={size[2]:9.3f}")

def main():
    folder = os.path.dirname(os.path.abspath(__file__))
    for path in sorted(glob.glob(os.path.join(folder, "*.stl"))):
        summarize(path)

if __name__ == "__main__":
    main()
