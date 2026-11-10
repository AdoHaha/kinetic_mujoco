"""Generator modelu MJCF (theisolab_motor.xml) wiernie odwzorowujący:
1. Przekrój techniczny (A - R00, B - Rotore 1, C - Rotore 2, A01-A04, N01-N03, P01-P03, C01-C06)
2. Geometrię pierścieni i 2x5 zakrzywionych ramion falowych z rysunków CAD autora (Scultura_cinetica_in_legno_01/02.jpg)
"""

import math
from pathlib import Path

SCRIPT_DIR = Path(__file__).resolve().parent
XML_OUT = SCRIPT_DIR / "theisolab_motor.xml"


def make_ring_geoms(prefix: str, y_pos: float, r_mid: float, half_width: float, half_thick: float, material: str, n_seg: int = 24) -> list[str]:
    """Tworzy gładki drewniany pierścień z n_seg segmentów pudełkowych w płaszczyźnie XZ."""
    lines = []
    dtheta = 2.0 * math.pi / n_seg
    seg_half_len = r_mid * math.tan(dtheta / 2.0) * 1.03
    for i in range(n_seg):
        theta = i * dtheta
        x = r_mid * math.sin(theta)
        z = r_mid * math.cos(theta)
        deg_y = math.degrees(theta)
        lines.append(
            f'      <geom name="{prefix}_ring_{i:02d}" type="box" '
            f'pos="{x:.5f} {y_pos:.5f} {z:.5f}" euler="0 {deg_y:.2f} 0" '
            f'size="{seg_half_len:.5f} {half_thick:.5f} {half_width:.5f}" '
            f'mass="0.0015" material="{material}" contype="0" conaffinity="0"/>'
        )
    return lines


def make_hub_spokes(prefix: str, y_pos: float, r_inner: float, r_outer: float, half_thick: float, material: str, chirality: int = 1) -> list[str]:
    """Tworzy 2 łukowate ramiona wewnętrzne łączące piastę N01 z pierścieniem rotora (jak na Scultura_cinetica_in_legno_02.jpg)."""
    lines = []
    for s_idx, base_angle_deg in enumerate([0.0, 180.0]):
        n_pts = 5
        radii = [r_inner + (r_outer - r_inner) * k / (n_pts - 1) for k in range(n_pts)]
        angles = [math.radians(base_angle_deg + chirality * (18.0 * math.sin(math.pi * k / (n_pts - 1)))) for k in range(n_pts)]
        for k in range(n_pts - 1):
            x1, z1 = radii[k] * math.sin(angles[k]), radii[k] * math.cos(angles[k])
            x2, z2 = radii[k + 1] * math.sin(angles[k + 1]), radii[k + 1] * math.cos(angles[k + 1])
            xm, zm = 0.5 * (x1 + x2), 0.5 * (z1 + z2)
            dx, dz = x2 - x1, z2 - z1
            seg_len = math.hypot(dx, dz) * 0.55
            deg_y = math.degrees(math.atan2(dx, dz))
            lines.append(
                f'      <geom name="{prefix}_spoke_{s_idx}_{k}" type="box" '
                f'pos="{xm:.5f} {y_pos:.5f} {zm:.5f}" euler="0 {deg_y:.2f} 0" '
                f'size="0.0055 {half_thick:.5f} {seg_len:.5f}" '
                f'mass="0.002" material="{material}" contype="0" conaffinity="0"/>'
            )
    return lines


def make_wave_blades(prefix: str, y_pos: float, r_ring: float, half_thick: float, material: str, chirality: int = 1) -> list[str]:
    """Tworzy 2 grupy po 5 zakrzywionych piór/fal (łącznie 10 ramion na rotor) zgodnie z CAD Theisolab."""
    lines = []
    # Dwie symetryczne sekcje po 5 piór (rozłożone co 22 stopnie w sekcji)
    for group_idx, group_base_deg in enumerate([0.0, 180.0]):
        for blade_idx in range(5):
            start_deg = group_base_deg - 44.0 + blade_idx * 22.0
            # Każde pióro ma krótkie ramię wyjściowe z pierścienia + długi łuk zewnętrzny
            pts_r = [r_ring, r_ring + 0.025, r_ring + 0.058, r_ring + 0.092, r_ring + 0.118, r_ring + 0.132]
            pts_deg = [
                start_deg,
                start_deg - chirality * 8.0,
                start_deg + chirality * 6.0,
                start_deg + chirality * 26.0,
                start_deg + chirality * 48.0,
                start_deg + chirality * 66.0,
            ]
            for k in range(len(pts_r) - 1):
                a1, a2 = math.radians(pts_deg[k]), math.radians(pts_deg[k + 1])
                x1, z1 = pts_r[k] * math.sin(a1), pts_r[k] * math.cos(a1)
                x2, z2 = pts_r[k + 1] * math.sin(a2), pts_r[k + 1] * math.cos(a2)
                xm, zm = 0.5 * (x1 + x2), 0.5 * (z1 + z2)
                dx, dz = x2 - x1, z2 - z1
                seg_half_len = math.hypot(dx, dz) * 0.54
                deg_y = math.degrees(math.atan2(dx, dz))
                w = 0.0042 if k < 4 else 0.0032
                lines.append(
                    f'      <geom name="{prefix}_blade_{group_idx}_{blade_idx}_{k}" type="box" '
                    f'pos="{xm:.5f} {y_pos:.5f} {zm:.5f}" euler="0 {deg_y:.2f} 0" '
                    f'size="{w:.5f} {half_thick:.5f} {seg_half_len:.5f}" '
                    f'mass="0.0016" material="{material}" contype="0" conaffinity="0"/>'
                )
    return lines


def make_star_teeth(prefix: str, y_pos: float, r_tooth: float, n_teeth: int = 12) -> list[str]:
    """Tworzy 12 zębów koła zapadkowego A03 - Dente (widocznego na Scultura_cinetica_in_legno_02.jpg pod nr 3)."""
    lines = []
    for i in range(n_teeth):
        deg = i * (360.0 / n_teeth)
        rad = math.radians(deg)
        x = r_tooth * math.sin(rad)
        z = r_tooth * math.cos(rad)
        lines.append(
            f'      <geom name="{prefix}_tooth_{i:02d}" type="box" '
            f'pos="{x:.5f} {y_pos:.5f} {z:.5f}" euler="0 {deg:.1f} 0" '
            f'size="0.0035 0.003 0.005" mass="0.0005" material="mat_dente_wood" contype="0" conaffinity="0"/>'
        )
    return lines


def build_xml() -> str:
    r1_ring = "\n".join(make_ring_geoms("R01", 0.0835, 0.080, 0.007, 0.004, "mat_rotor1_wood", n_seg=24))
    r1_spokes = "\n".join(make_hub_spokes("R01", 0.0835, 0.022, 0.076, 0.004, "mat_rotor1_wood", chirality=1))
    r1_blades = "\n".join(make_wave_blades("R01", 0.0835, 0.084, 0.004, "mat_rotor1_wood", chirality=1))

    r2_ring = "\n".join(make_ring_geoms("R02", 0.1280, 0.080, 0.007, 0.004, "mat_rotor2_wood", n_seg=24))
    r2_spokes = "\n".join(make_hub_spokes("R02", 0.1280, 0.022, 0.076, 0.004, "mat_rotor2_wood", chirality=-1))
    r2_blades = "\n".join(make_wave_blades("R02", 0.1280, 0.084, 0.004, "mat_rotor2_wood", chirality=-1))

    a03_teeth = "\n".join(make_star_teeth("A03", 0.0535, 0.036, n_teeth=12))

    xml = f"""<mujoco model="theisolab_kinetic_motor">
  <compiler angle="degree" coordinate="local" inertiafromgeom="true"/>
  <option timestep="0.001" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" impratio="10"/>

  <visual>
    <global offwidth="1920" offheight="640" azimuth="70" elevation="-12"/>
    <headlight ambient="0.42 0.42 0.42" diffuse="0.75 0.75 0.75" specular="0.35 0.35 0.35"/>
    <rgba haze="0.12 0.14 0.18 1"/>
  </visual>

  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="0.14 0.16 0.20" rgb2="0.19 0.22 0.27"/>
    <material name="mat_floor" texture="grid" texrepeat="6 6" reflectance="0.08"/>
    <material name="mat_wall_wood" rgba="0.68 0.43 0.28 1.0" specular="0.2" shininess="0.25"/>
    <material name="mat_spool_wood" rgba="0.86 0.64 0.40 1.0" specular="0.25" shininess="0.3"/>
    <material name="mat_dente_wood" rgba="0.74 0.46 0.25 1.0" specular="0.25" shininess="0.3"/>
    <material name="mat_rotor1_wood" rgba="0.76 0.54 0.34 1.0" specular="0.3" shininess="0.35"/>
    <material name="mat_rotor2_wood" rgba="0.88 0.68 0.44 1.0" specular="0.3" shininess="0.35"/>
    <material name="mat_hub_wood" rgba="0.72 0.45 0.26 1.0" specular="0.25" shininess="0.3"/>
    <material name="mat_steel" rgba="0.78 0.80 0.85 1.0" specular="0.85" shininess="0.85"/>
    <material name="mat_bearing" rgba="0.58 0.63 0.70 1.0" specular="0.9" shininess="0.9"/>
    <material name="mat_cord_coil" rgba="0.32 0.36 0.42 1.0" specular="0.6" shininess="0.6"/>
  </asset>

  <default>
    <geom solref="0.003 1.0" solimp="0.95 0.99 0.001" friction="0.08 0.002 0.0001"/>
    <joint damping="0.0005" armature="0.0002"/>
  </default>

  <worldbody>
    <light pos="0.3 0.6 0.7" dir="-0.2 -0.6 -0.5" directional="true" castshadow="true"/>
    <light pos="-0.4 0.5 0.5" dir="0.3 -0.5 -0.4" directional="false" diffuse="0.5 0.5 0.5"/>
    <geom name="floor" type="plane" pos="0 0 -0.28" size="0.8 0.8 0.01" material="mat_floor" contype="0" conaffinity="0"/>

    <!-- ========================================================================= -->
    <!-- SEKCJA (A): R00 - Supporto a parete (okrągła tarcza ścienna jak w CAD)    -->
    <!-- + C01 - Vite filettata (M8) + P01 - Perno metallico (D2 6mm L 60mm)       -->
    <!-- ========================================================================= -->
    <body name="R00_supporto_parete" pos="0 0 0">
      <!-- R00: Okrągła tarcza ścienna (A) przy y = 0 (zgodnie z Scultura_cinetica_in_legno_01.jpg) -->
      <geom name="R00_disc" type="cylinder" pos="0 0 0" euler="90 0 0" size="0.082 0.008" material="mat_wall_wood" contype="0" conaffinity="0"/>

      <!-- C02 - Vite da falegname + C01 - Vite filettata (M8, r=4mm, L=175mm) -->
      <geom name="C02_head" type="cylinder" pos="0 -0.0085 0" euler="90 0 0" size="0.011 0.0015" material="mat_steel" contype="0" conaffinity="0"/>
      <geom name="C01_shaft_M8" type="cylinder" pos="0 0.080 0" euler="90 0 0" size="0.004 0.086" material="mat_steel" contype="0" conaffinity="0"/>

      <!-- C03 - Rondella (S 3mm D1 8mm D2 20mm) + A05 - Dado M8 -->
      <geom name="C03_rondella" type="cylinder" pos="0 0.0095 0" euler="90 0 0" size="0.010 0.0015" material="mat_steel" contype="0" conaffinity="0"/>
      <geom name="A05_dado_M8" type="cylinder" pos="0 0.0145 0" euler="90 0 0" size="0.0075 0.0035" material="mat_steel" contype="0" conaffinity="0"/>

      <!-- C05 - Dado M8 (nakrętka końcowa z prawej strony wału przy y = 0.158) -->
      <geom name="C05_dado_M8" type="cylinder" pos="0 0.158 0" euler="90 0 0" size="0.0075 0.004" material="mat_steel" contype="0" conaffinity="0"/>

      <!-- Podkładki dystansowe na osi M8: Rondella (S 2mm D1 8mm D2 14mm) & C04 -->
      <geom name="washer_1" type="cylinder" pos="0 0.0195 0" euler="90 0 0" size="0.007 0.001" material="mat_steel" contype="0" conaffinity="0"/>
      <geom name="washer_2" type="cylinder" pos="0 0.064 0" euler="90 0 0" size="0.007 0.001" material="mat_steel" contype="0" conaffinity="0"/>
      <geom name="washer_3" type="cylinder" pos="0 0.104 0" euler="90 0 0" size="0.007 0.001" material="mat_steel" contype="0" conaffinity="0"/>
      <geom name="C04_washer_4" type="cylinder" pos="0 0.120 0" euler="90 0 0" size="0.007 0.001" material="mat_steel" contype="0" conaffinity="0"/>
      <geom name="C04_washer_5" type="cylinder" pos="0 0.152 0" euler="90 0 0" size="0.007 0.001" material="mat_steel" contype="0" conaffinity="0"/>

      <!-- N03 - Nottolino distanziatore (tuleja dystansowa na wale między B i C) -->
      <geom name="N03_spacer" type="cylinder" pos="0 0.112 0" euler="90 0 0" size="0.009 0.007" material="mat_hub_wood" contype="0" conaffinity="0"/>

      <!-- P01 - Perno metallico (D2 6mm L 60mm) osadzony w R00 (A) u góry na promieniu r = 49 mm -->
      <geom name="P01_perno_stop" type="cylinder" pos="0 0.025 0.049" euler="90 0 0" size="0.003 0.025" material="mat_steel" contype="0" conaffinity="0"/>
      <!-- Górny A03 - Dente (zapadka stacjonarna osadzona na trzpieniu P01 przy y = 0.046 m, zazębiona z kołem A03) -->
      <geom name="A03_pawl_upper_on_P01" type="box" pos="0 0.046 0.045" size="0.005 0.004 0.008" material="mat_dente_wood" contype="0" conaffinity="0"/>
    </body>

    <!-- ========================================================================= -->
    <!-- ZESPÓŁ BĘBNA NAWIJAJĄCEGO I KOŁA ZĘBATEGO: A01-1, A02, A01-2, A03, N01    -->
    <!-- ========================================================================= -->
    <body name="A02_spool_assembly" pos="0 0 0">
      <joint name="joint_spool" type="hinge" axis="0 1 0" damping="0.015"/>

      <!-- Dwa łożyska C06 (608ZZ: r=11mm, B=7mm) i oprawa N01 -->
      <geom name="C06_bearing_spool_L" type="cylinder" pos="0 0.0245 0" euler="90 0 0" size="0.011 0.0035" material="mat_bearing" contype="0" conaffinity="0"/>
      <geom name="C06_bearing_spool_R" type="cylinder" pos="0 0.0595 0" euler="90 0 0" size="0.011 0.0035" material="mat_bearing" contype="0" conaffinity="0"/>
      <geom name="N01_spool_R" type="cylinder" pos="0 0.0595 0" euler="90 0 0" size="0.018 0.004" material="mat_hub_wood" contype="0" conaffinity="0"/>

      <!-- A01-1: lewy kołnierz bębna, A02: rdzeń bębna + nawinięta sprężyna/linka, A01-2: prawy kołnierz -->
      <geom name="A01_1_flange_L" type="cylinder" pos="0 0.0245 0" euler="90 0 0" size="0.048 0.003" mass="0.025" material="mat_spool_wood" contype="0" conaffinity="0"/>
      <geom name="A02_spool_core" type="cylinder" pos="0 0.0365 0" euler="90 0 0" size="0.035 0.009" mass="0.040" material="mat_spool_wood" contype="0" conaffinity="0"/>
      <geom name="A02_wound_coil" type="cylinder" pos="0 0.0365 0" euler="90 0 0" size="0.038 0.007" mass="0.010" material="mat_cord_coil" contype="0" conaffinity="0"/>
      <geom name="A01_2_flange_R" type="cylinder" pos="0 0.0480 0" euler="90 0 0" size="0.048 0.0025" mass="0.020" material="mat_spool_wood" contype="0" conaffinity="0"/>

      <!-- A03 - Dente (koło zębate 12-zębne + zabierak dolny na y = 0.0535 m) -->
      <geom name="A03_disc" type="cylinder" pos="0 0.0535 0" euler="90 0 0" size="0.034 0.003" mass="0.020" material="mat_dente_wood" contype="0" conaffinity="0"/>
{a03_teeth}
      <!-- Dolny A03 - Dente (zabierak na y = 0.0535 m sięgający do r = 64 mm, popychający P02 su rotore 1) -->
      <geom name="A03_dente_bottom" type="box" pos="0 0.0535 -0.050" size="0.005 0.0035 0.015" mass="0.008" material="mat_dente_wood" contype="1" conaffinity="1"/>
    </body>

    <!-- ========================================================================= -->
    <!-- SEKCJA (B): R01 - Rotore 1 + N01 + C06 + P02 (4mm i zabierakowy)          -->
    <!-- ========================================================================= -->
    <body name="R01_rotore_1" pos="0 0 0">
      <joint name="joint_rotore1" type="hinge" axis="0 1 0" damping="0.004"/>

      <!-- Dwa łożyska C06 i dwie oprawy N01 - Nottolino per cuscinetto po bokach R01 -->
      <geom name="C06_bearing_R1_L" type="cylinder" pos="0 0.070 0" euler="90 0 0" size="0.011 0.0035" material="mat_bearing" contype="0" conaffinity="0"/>
      <geom name="N01_R1_L" type="cylinder" pos="0 0.070 0" euler="90 0 0" size="0.024 0.005" material="mat_hub_wood" contype="0" conaffinity="0"/>

      <geom name="C06_bearing_R1_R" type="cylinder" pos="0 0.097 0" euler="90 0 0" size="0.011 0.0035" material="mat_bearing" contype="0" conaffinity="0"/>
      <geom name="N01_R1_R" type="cylinder" pos="0 0.097 0" euler="90 0 0" size="0.024 0.005" material="mat_hub_wood" contype="0" conaffinity="0"/>

      <!-- P02 - Perno metallico (D2 4mm L 40mm): 4 trzpienie przechodzące przez N01-L, R01, N01-R -->
      <geom name="P02_pin_1" type="capsule" pos="0.020 0.095 0" euler="90 0 0" size="0.0022 0.020" material="mat_steel" contype="2" conaffinity="2"/>
      <geom name="P02_pin_2" type="capsule" pos="-0.020 0.095 0" euler="90 0 0" size="0.0022 0.020" material="mat_steel" contype="2" conaffinity="2"/>
      <geom name="P02_pin_3" type="capsule" pos="0 0.095 0.020" euler="90 0 0" size="0.0022 0.020" material="mat_steel" contype="0" conaffinity="0"/>
      <geom name="P02_pin_4" type="capsule" pos="0 0.095 -0.020" euler="90 0 0" size="0.0022 0.020" material="mat_steel" contype="0" conaffinity="0"/>

      <!-- P02 - Perno metallico su rotore 1 (na dole, większy promień r = 62 mm jak na rysunku przekrojowym, -->
      <!-- dzięki czemu przy obrocie mija górny trzpień P01 [r = 49 mm] oraz górną zapadkę bez kolizji!) -->
      <geom name="P02_su_rotore1" type="capsule" pos="-0.009 0.068 -0.062" euler="90 0 0" size="0.003 0.015" material="mat_steel" contype="1" conaffinity="1"/>

      <!-- Pierścień nośny, 2 łukowate szprychy i 10 zakrzywionych piór falowych R01 - Rotore 1 (B) -->
{r1_ring}
{r1_spokes}
{r1_blades}
    </body>

    <!-- ========================================================================= -->
    <!-- SEKCJA (C): R02 - Rotore 2 + N02/A04 (uchylny nottolino) + P03 + N01      -->
    <!-- ========================================================================= -->
    <body name="R02_rotore_2" pos="0 0 0">
      <joint name="joint_rotore2" type="hinge" axis="0 1 0" damping="0.0010"/>

      <!-- Dwa łożyska C06 (jedno w R02 przy y=0.126, drugie w N01 przy y=0.147) -->
      <geom name="C06_bearing_R2_L" type="cylinder" pos="0 0.126 0" euler="90 0 0" size="0.011 0.0035" material="mat_bearing" contype="0" conaffinity="0"/>
      <geom name="C06_bearing_R2_R" type="cylinder" pos="0 0.147 0" euler="90 0 0" size="0.011 0.0035" material="mat_bearing" contype="0" conaffinity="0"/>
      <!-- Przednia oprawa łożyska N01 (widoczna w centrum z przodu na Scultura_cinetica_in_legno_01.jpg) -->
      <geom name="N01_R2_R" type="cylinder" pos="0 0.140 0" euler="90 0 0" size="0.025 0.007" material="mat_hub_wood" contype="0" conaffinity="0"/>
      <geom name="N01_R2_cap" type="cylinder" pos="0 0.151 0" euler="90 0 0" size="0.017 0.005" material="mat_wall_wood" contype="0" conaffinity="0"/>

      <!-- Pierścień nośny, 2 łukowate szprychy i 10 zakrzywionych piór falowych R02 - Rotore 2 (C, przeciwny zwrot!) -->
{r2_ring}
{r2_spokes}
{r2_blades}

      <!-- Dyskretny ciężarek wyważający (contrappeso) po wewnętrznej stronie pierścienia R02 (zgodnie z artykułem Theisolab) -->
      <geom name="R02_contrappeso" type="cylinder" pos="0 0.124 -0.078" euler="90 0 0" size="0.010 0.004" mass="0.048" material="mat_steel" contype="0" conaffinity="0"/>

      <!-- C04 - Rondella (S 2mm D1 2mm D2 6mm) pod zapadką N02/A04 -->
      <geom name="C04_nottolino_washer" type="cylinder" pos="0 0.121 0.034" euler="90 0 0" size="0.004 0.001" material="mat_steel" contype="0" conaffinity="0"/>

      <!-- N02 / A04 - Nottolino (uchylna zapadka na śrubie z trzpieniami P03 - Perno metallico D2 1.5 L 27mm) -->
      <body name="N02_nottolino_pawl" pos="0 0.115 0.034">
        <joint name="joint_nottolino_N02" type="hinge" axis="0 1 0" springref="0" stiffness="0.055" damping="0.0008" limited="true" range="-65 35"/>
        <geom name="N02_body" type="box" pos="0 0 -0.006" size="0.006 0.004 0.010" mass="0.005" material="mat_spool_wood" contype="0" conaffinity="0"/>
        <!-- P03 - Perno metallico (D2 1,5 L 27mm) sięgające na promień r = 0.0195 m (zasięg trzpieni P02) -->
        <geom name="P03_pin_catch" type="capsule" pos="0 -0.004 -0.0145" euler="90 0 0" size="0.0020 0.0135" mass="0.002" material="mat_steel" contype="2" conaffinity="2"/>
        <geom name="P03_pin_guide" type="capsule" pos="0 -0.004 -0.010" euler="90 0 0" size="0.0012 0.0135" mass="0.001" material="mat_steel" contype="0" conaffinity="0"/>
      </body>
    </body>
  </worldbody>

  <!-- Stały moment napędowy na bębnie A02 (Molla a tensione costante / naciąg nawinięty na A02) -->
  <actuator>
    <motor name="spring_drive_A02" joint="joint_spool" gear="1" ctrlrange="0 0.1" ctrllimited="true"/>
  </actuator>
</mujoco>
"""
    XML_OUT.write_text(xml, encoding="utf-8")
    print(f"Wygenerowano model: {XML_OUT}")


def main() -> None:
    build_xml()


if __name__ == "__main__":
    main()
