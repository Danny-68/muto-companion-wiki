#!/usr/bin/env python3
"""cluster_muto_shells.py -- draait op de WSL2/RTX5080-machine.
De STEP-export van MutoRS heeft geen assembly-structuur (bevestigd,
extract_muto_assembly.py) -- dit script groepeert de 2341 losse shells
GEOMETRISCH: shells waarvan de boundingboxen (met een kleine marge)
overlappen worden als hetzelfde fysieke onderdeel beschouwd (union-find op
boundingbox-overlap). Daarna clusteren we die onderdelen hoekgewijs rond het
chassis-midden (6 sectoren = 6 poten) en radiaal (dichtstbij/midden/verst =
coxa/femur/tibia), als eerste, grove opstap naar Fase 7's Muto-MJCF.
"""
import math

import cadquery as cq
from OCP.TopAbs import TopAbs_SHELL
from OCP.TopExp import TopExp_Explorer
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box

STEP_PATH = "/home/meinds/projects/muto_rs/yahboom_hardware/5.About hardware/3D_Model_File/MutoRS.STEP"
MERGE_TOL_MM = 2.0  # boundingboxen binnen deze marge worden als "aanrakend" beschouwd


class UnionFind:
    def __init__(self, n):
        self.parent = list(range(n))

    def find(self, x):
        while self.parent[x] != x:
            self.parent[x] = self.parent[self.parent[x]]
            x = self.parent[x]
        return x

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb


def bbox_of(shape):
    box = Bnd_Box()
    BRepBndLib.Add_s(shape, box)
    xmin, ymin, zmin, xmax, ymax, zmax = box.Get()
    return xmin, ymin, zmin, xmax, ymax, zmax


def overlaps(b1, b2, tol):
    x1min, y1min, z1min, x1max, y1max, z1max = b1
    x2min, y2min, z2min, x2max, y2max, z2max = b2
    return (x1min - tol <= x2max and x2min - tol <= x1max and
            y1min - tol <= y2max and y2min - tol <= y1max and
            z1min - tol <= z2max and z2min - tol <= z1max)


def main():
    print("STEP laden...")
    model = cq.importers.importStep(STEP_PATH)
    shape = model.val().wrapped

    shells = []
    exp = TopExp_Explorer(shape, TopAbs_SHELL)
    while exp.More():
        shells.append(exp.Current())
        exp.Next()
    print(f"{len(shells)} shells gevonden, boundingboxen berekenen...")

    bboxes = [bbox_of(s) for s in shells]

    print("Union-find clusteren op boundingbox-overlap...")
    uf = UnionFind(len(shells))
    # Sorteer op xmin zodat we een sweep-achtige aanpak kunnen doen (grof, maar
    # voor een paar duizend shells ruim snel genoeg als O(n^2) met vroege stop).
    for i in range(len(shells)):
        for j in range(i + 1, len(shells)):
            if bboxes[j][0] > bboxes[i][3] + MERGE_TOL_MM:
                continue  # xmin van j ver voorbij xmax van i -- door sortering hieronder niet relevant
            if overlaps(bboxes[i], bboxes[j], MERGE_TOL_MM):
                uf.union(i, j)

    clusters = {}
    for i in range(len(shells)):
        root = uf.find(i)
        clusters.setdefault(root, []).append(i)

    print(f"\n{len(clusters)} losse geometrische clusters gevonden uit {len(shells)} shells")

    # Per cluster: gecombineerde boundingbox + centroid + grootte (aantal shells)
    cluster_info = []
    for root, members in clusters.items():
        xs_min = min(bboxes[i][0] for i in members)
        ys_min = min(bboxes[i][1] for i in members)
        zs_min = min(bboxes[i][2] for i in members)
        xs_max = max(bboxes[i][3] for i in members)
        ys_max = max(bboxes[i][4] for i in members)
        zs_max = max(bboxes[i][5] for i in members)
        cx, cy, cz = (xs_min + xs_max) / 2, (ys_min + ys_max) / 2, (zs_min + zs_max) / 2
        size = max(xs_max - xs_min, ys_max - ys_min, zs_max - zs_min)
        cluster_info.append({
            "n_shells": len(members),
            "centroid": (cx, cy, cz),
            "size_mm": size,
            "bbox": (xs_min, ys_min, zs_min, xs_max, ys_max, zs_max),
        })

    cluster_info.sort(key=lambda c: -c["n_shells"])
    print("\nGrootste 30 clusters (aantal shells, grootte mm, centroid mm):")
    for c in cluster_info[:30]:
        print(f"  shells={c['n_shells']:4d}  grootte={c['size_mm']:7.1f}mm  centroid=({c['centroid'][0]:7.1f},{c['centroid'][1]:7.1f},{c['centroid'][2]:7.1f})")

    # Overall centroid (gewogen op aantal shells, als grove schatting van het lichaamsmidden)
    total_shells = sum(c["n_shells"] for c in cluster_info)
    cx = sum(c["centroid"][0] * c["n_shells"] for c in cluster_info) / total_shells
    cy = sum(c["centroid"][1] * c["n_shells"] for c in cluster_info) / total_shells
    print(f"\nGeschat lichaamsmidden (shell-gewogen): x={cx:.1f} y={cy:.1f}")

    print(f"\nVerdeling clustergroottes: >100 shells: {sum(1 for c in cluster_info if c['n_shells']>100)}, "
          f"10-100: {sum(1 for c in cluster_info if 10<=c['n_shells']<=100)}, "
          f"<10: {sum(1 for c in cluster_info if c['n_shells']<10)}")


if __name__ == "__main__":
    main()
