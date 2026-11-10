"""Ekstrakcja ścieżek wektorowych z plików PDF:
- documents/Mandalka_mala2a.pdf (przednie skrzydła - Rotore 2 / C)
- documents/Mandalka_mala2b.pdf (tylne skrzydła - Rotore 1 / B)
i generowanie siatek 3D STL oraz osobnego modelu MuJoCo 4 (theisolab_mandalka.xml).
"""

from pathlib import Path
import re
import struct
import zlib
import numpy as np

SCRIPT_DIR = Path(__file__).resolve().parent
DOCS_DIR = (SCRIPT_DIR.parent / "documents") if (SCRIPT_DIR.parent / "documents").exists() else (SCRIPT_DIR.parent.parent / "documents")
MESH_DIR = SCRIPT_DIR / "meshes"
XML_OUT = SCRIPT_DIR / "theisolab_mandalka.xml"


def extract_pdf_polygon(pdf_path: Path, samples_per_curve: int = 8, target_outer_radius: float = 0.220) -> np.ndarray:
    """Wypakowuje strumień FlateDecode z PDF i próbkuje główny kontur Béziera do wielokąta 2D (x, z) w metrach."""
    raw = pdf_path.read_bytes()
    streams = re.findall(b"stream\r?\n(.*?)\r?\nendstream", raw, flags=re.S)
    dec = zlib.decompress(streams[0]).decode("latin1")
    first_subpath = dec.split("h")[0]

    tokens = first_subpath.split()
    pts = []
    idx = 0
    cur = np.zeros(2, dtype=float)

    while idx < len(tokens):
        tok = tokens[idx]
        if tok in ("q", "/a0", "gs"):
            idx += 1
        elif tok == "cm":
            idx += 1
        elif tok == "rg":
            idx += 1
        elif tok == "m":
            x = float(tokens[idx - 2])
            y = float(tokens[idx - 1])
            cur = np.array([x, y], dtype=float)
            pts.append(cur.copy())
            idx += 1
        elif tok == "c":
            x1, y1 = float(tokens[idx - 6]), float(tokens[idx - 5])
            x2, y2 = float(tokens[idx - 4]), float(tokens[idx - 3])
            x3, y3 = float(tokens[idx - 2]), float(tokens[idx - 1])
            p0 = cur
            p1 = np.array([x1, y1], dtype=float)
            p2 = np.array([x2, y2], dtype=float)
            p3 = np.array([x3, y3], dtype=float)
            for s in range(1, samples_per_curve + 1):
                t = s / samples_per_curve
                u = 1.0 - t
                pt = (u**3) * p0 + 3 * (u**2) * t * p1 + 3 * u * (t**2) * p2 + (t**3) * p3
                pts.append(pt)
            cur = p3
            idx += 1
        else:
            idx += 1

    poly = np.array(pts, dtype=float)
    # Usuń duplikaty sąsiednich punktów
    cleaned = [poly[0]]
    for p in poly[1:]:
        if np.linalg.norm(p - cleaned[-1]) > 1e-2:
            cleaned.append(p)
    if np.linalg.norm(cleaned[0] - cleaned[-1]) <= 1e-2:
        cleaned.pop()
    poly = np.array(cleaned, dtype=float)

    # Wyśrodkowanie względem geometrycznego środka tarczy (164.752, 164.532) i przeskalowanie
    center = np.array([164.75195, 164.53221], dtype=float)
    poly[:, 0] = poly[:, 0] - center[0]
    # W PDF transformacja '1 0 0 -1 0 329.064 cm' odwraca oś Y
    poly[:, 1] = -(poly[:, 1] - center[1])

    max_r = np.max(np.linalg.norm(poly, axis=1))
    scale = target_outer_radius / max_r
    poly *= scale
    return poly


def signed_area(poly: np.ndarray) -> float:
    x = poly[:, 0]
    y = poly[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def point_in_triangle(pt: np.ndarray, a: np.ndarray, b: np.ndarray, c: np.ndarray) -> bool:
    v0 = c - a
    v1 = b - a
    v2 = pt - a
    dot00 = np.dot(v0, v0)
    dot01 = np.dot(v0, v1)
    dot02 = np.dot(v0, v2)
    dot11 = np.dot(v1, v1)
    dot12 = np.dot(v1, v2)
    denom = dot00 * dot11 - dot01 * dot01
    if abs(denom) < 1e-14:
        return False
    inv = 1.0 / denom
    u = (dot11 * dot02 - dot01 * dot12) * inv
    v = (dot00 * dot12 - dot01 * dot02) * inv
    return (u >= -1e-9) and (v >= -1e-9) and (u + v <= 1.0 + 1e-9)


def ear_clip_triangulate(poly: np.ndarray) -> list[tuple[int, int, int]]:
    """Triangulacja wielokąta prostego metodą ear-clipping (zorientowanego przeciwnie do ruchu wskazówek zegara)."""
    if signed_area(poly) < 0:
        poly = poly[::-1].copy()

    indices = list(range(len(poly)))
    triangles: list[tuple[int, int, int]] = []

    def is_ear(i_prev: int, i_curr: int, i_next: int, rem: list[int]) -> bool:
        a, b, c = poly[i_prev], poly[i_curr], poly[i_next]
        cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
        if cross <= 1e-11:
            return False
        for idx in rem:
            if idx in (i_prev, i_curr, i_next):
                continue
            if point_in_triangle(poly[idx], a, b, c):
                return False
        return True

    guard = 0
    while len(indices) > 3 and guard < 20000:
        guard += 1
        n = len(indices)
        ear_found = False
        # Wybierz ucho o największej wypukłości / najlepszym kącie
        for k in range(n):
            i_prev = indices[(k - 1) % n]
            i_curr = indices[k]
            i_next = indices[(k + 1) % n]
            if is_ear(i_prev, i_curr, i_next, indices):
                triangles.append((i_prev, i_curr, i_next))
                indices.pop(k)
                ear_found = True
                break
        if not ear_found:
            # Awaryjne odcięcie najbardziej wypukłego wierzchołka
            best_k = 0
            best_cross = -1e9
            for k in range(n):
                a = poly[indices[(k - 1) % n]]
                b = poly[indices[k]]
                c = poly[indices[(k + 1) % n]]
                cross = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
                if cross > best_cross:
                    best_cross = cross
                    best_k = k
            i_prev = indices[(best_k - 1) % n]
            i_curr = indices[best_k]
            i_next = indices[(best_k + 1) % n]
            triangles.append((i_prev, i_curr, i_next))
            indices.pop(best_k)

    if len(indices) == 3:
        triangles.append((indices[0], indices[1], indices[2]))

    return poly, triangles


def write_extruded_stl(poly: np.ndarray, triangles: list[tuple[int, int, int]], half_thick_y: float, out_path: Path) -> None:
    """Zapisuje wytłoczoną wzdłuż osi Y siatkę trójkątów (w płaszczyźnie XZ, grubość po Y) jako binarny plik STL."""
    all_tris = []

    # Przednia i tylna ściana (wzdłuż osi Y: y = -half_thick_y oraz y = +half_thick_y)
    for i0, i1, i2 in triangles:
        p0, p1, p2 = poly[i0], poly[i1], poly[i2]
        # Ściana +Y (normalna [0, +1, 0])
        v0 = (p0[0], half_thick_y, p0[1])
        v1 = (p2[0], half_thick_y, p2[1])
        v2 = (p1[0], half_thick_y, p1[1])
        all_tris.append(((0.0, 1.0, 0.0), v0, v1, v2))

        # Ściana -Y (normalna [0, -1, 0])
        u0 = (p0[0], -half_thick_y, p0[1])
        u1 = (p1[0], -half_thick_y, p1[1])
        u2 = (p2[0], -half_thick_y, p2[1])
        all_tris.append(((0.0, -1.0, 0.0), u0, u1, u2))

    # Ścianki boczne obwodu
    n = len(poly)
    for k in range(n):
        p0 = poly[k]
        p1 = poly[(k + 1) % n]
        dx = p1[0] - p0[0]
        dz = p1[1] - p0[1]
        length = max(1e-9, float(np.hypot(dx, dz)))
        nx, nz = dz / length, -dx / length

        a = (p0[0], -half_thick_y, p0[1])
        b = (p1[0], -half_thick_y, p1[1])
        c = (p1[0], half_thick_y, p1[1])
        d = (p0[0], half_thick_y, p0[1])

        all_tris.append(((nx, 0.0, nz), a, b, c))
        all_tris.append(((nx, 0.0, nz), a, c, d))

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "wb") as f:
        header = b"Mandalka Rotor STL generated for MuJoCo 4.0".ljust(80, b" ")
        f.write(header)
        f.write(struct.pack("<I", len(all_tris)))
        for norm, v0, v1, v2 in all_tris:
            f.write(struct.pack("<12fH", *norm, *v0, *v1, *v2, 0))


def main() -> None:
    for name, pdf_name in [("mandalka_2a.stl", "Mandalka_mala2a.pdf"), ("mandalka_2b.stl", "Mandalka_mala2b.pdf")]:
        poly = extract_pdf_polygon(DOCS_DIR / pdf_name, samples_per_curve=6, target_outer_radius=0.220)
        poly_ccw, tris = ear_clip_triangulate(poly)
        stl_path = MESH_DIR / name
        write_extruded_stl(poly_ccw, tris, half_thick_y=0.004, out_path=stl_path)
        print(f"{pdf_name} -> {stl_path} (wierzcholki 2D: {len(poly_ccw)}, trojkaty 2D: {len(tris)})")


if __name__ == "__main__":
    main()
