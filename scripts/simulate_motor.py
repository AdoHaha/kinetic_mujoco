import argparse
from pathlib import Path
import time
import imageio.v2 as imageio
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
MODEL_PATH = SCRIPT_DIR / "theisolab_motor.xml"
VIDEO_PATH = SCRIPT_DIR / "theisolab_simulation.mp4"
PLOT_PATH = SCRIPT_DIR / "theisolab_kinematics.png"
PREVIEW_FRAME_PATH = SCRIPT_DIR / "theisolab_preview.png"

# Stały moment napędowy na bębnie A02 (Molla a tensione costante / naciąg)
DRIVE_TORQUE = 0.055


def run_headless_recording(duration: float = 10.0, fps: int = 60) -> None:
    print(f"[MuJoCo {mujoco.__version__}] Wczytywanie modelu: {MODEL_PATH}")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)

    mujoco.mj_resetData(model, data)
    data.ctrl[0] = DRIVE_TORQUE
    mujoco.mj_forward(model, data)

    # Indeksy przegubów
    jid_spool = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_spool")
    jid_r1 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_rotore1")
    jid_r2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_rotore2")
    jid_n02 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "joint_nottolino_N02")

    qpos_spool = model.jnt_qposadr[jid_spool]
    qpos_r1 = model.jnt_qposadr[jid_r1]
    qpos_r2 = model.jnt_qposadr[jid_r2]
    qpos_n02 = model.jnt_qposadr[jid_n02]

    dof_r1 = model.jnt_dofadr[jid_r1]
    dof_r2 = model.jnt_dofadr[jid_r2]

    # Renderowanie 3 widoków obok siebie:
    # 1. FRONT (od przodu wzdłuż osi Y)
    # 2. IZOMETRYCZNY 3D (jak na rzucie CAD Scultura_cinetica_in_legno_01.jpg)
    # 3. BOK / PRZEKRÓJ A-B-C (jak na rysunku technicznym Zrzut ekranu 2026-11-09 163301.png)
    width, height = 640, 544
    renderer = mujoco.Renderer(model, height=height, width=width)

    cam_front = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam_front)
    cam_front.lookat[:] = [0.0, 0.10, 0.0]
    cam_front.distance = 0.52
    cam_front.azimuth = -90.0
    cam_front.elevation = 0.0

    cam_iso = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam_iso)
    cam_iso.lookat[:] = [0.0, 0.085, 0.0]
    cam_iso.distance = 0.50
    cam_iso.azimuth = -55.0
    cam_iso.elevation = -14.0

    cam_side = mujoco.MjvCamera()
    mujoco.mjv_defaultFreeCamera(model, cam_side)
    cam_side.lookat[:] = [0.0, 0.080, 0.0]
    cam_side.distance = 0.25
    cam_side.azimuth = 180.0
    cam_side.elevation = -2.0

    times = []
    hist_spool_deg = []
    hist_r1_deg = []
    hist_r2_deg = []
    hist_n02_deg = []
    hist_w1 = []
    hist_w2 = []

    steps_per_frame = max(1, int(round(1.0 / (fps * model.opt.timestep))))
    total_steps = int(round(duration / model.opt.timestep))

    writer = imageio.get_writer(str(VIDEO_PATH), fps=fps, codec="libx264", quality=8)

    for step in range(total_steps):
        data.ctrl[0] = DRIVE_TORQUE
        mujoco.mj_step(model, data)

        if step % 10 == 0:
            times.append(data.time)
            hist_spool_deg.append(np.degrees(data.qpos[qpos_spool]))
            hist_r1_deg.append(np.degrees(data.qpos[qpos_r1]))
            hist_r2_deg.append(np.degrees(data.qpos[qpos_r2]))
            hist_n02_deg.append(np.degrees(data.qpos[qpos_n02]))
            hist_w1.append(np.degrees(data.qvel[dof_r1]))
            hist_w2.append(np.degrees(data.qvel[dof_r2]))

        if step % steps_per_frame == 0:
            renderer.update_scene(data, camera=cam_front)
            frame_front = renderer.render().copy()

            renderer.update_scene(data, camera=cam_iso)
            frame_iso = renderer.render().copy()

            renderer.update_scene(data, camera=cam_side)
            frame_side = renderer.render().copy()

            combined = np.hstack([frame_front, frame_iso, frame_side])
            writer.append_data(combined)

            if step == steps_per_frame * 60:
                imageio.imwrite(str(PREVIEW_FRAME_PATH), combined)

    writer.close()
    renderer.close()

    print(f"Zapisano wideo symulacji (Front | 3D | Bok A-B-C): {VIDEO_PATH}")
    print(
        f"Podsumowanie po {duration:.1f}s: "
        f"Spool={hist_spool_deg[-1]:.1f} deg, "
        f"Rotore 1={hist_r1_deg[-1]:.1f} deg, "
        f"Rotore 2 (min/max)=({min(hist_r2_deg):.1f}, {max(hist_r2_deg):.1f}) deg, "
        f"N02 pawl (min/max)=({min(hist_n02_deg):.1f}, {max(hist_n02_deg):.1f}) deg"
    )

    # Wykresy kinematyki
    fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    axes[0].plot(times, hist_spool_deg, label="A02 Nottolino avvolgi filo (bęben)", linestyle="--", color="#d97706")
    axes[0].plot(times, hist_r1_deg, label="R01 - Rotore 1 (B)", color="#b45309", linewidth=2)
    axes[0].plot(times, hist_r2_deg, label="R02 - Rotore 2 (C)", color="#2563eb", linewidth=2)
    axes[0].set_ylabel("Kąt obrotu [deg]")
    axes[0].set_title("Symulacja silnika rzeźby kinetycznej Theisolab w MuJoCo 4.0")
    axes[0].legend(loc="upper left")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, hist_w1, label="ω Rotore 1 [deg/s]", color="#b45309", alpha=0.85)
    axes[1].plot(times, hist_w2, label="ω Rotore 2 [deg/s] (ruch impulsowy / przeciwbieżny)", color="#2563eb", alpha=0.85)
    axes[1].axhline(0, color="black", linewidth=0.8, linestyle=":")
    axes[1].set_ylabel("Prędkość kątowa [deg/s]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].plot(times, hist_n02_deg, label="Ugięcie zapadki N02 / P03 [deg]", color="#059669")
    axes[2].set_xlabel("Czas [s]")
    axes[2].set_ylabel("Kąt zapadki N02 [deg]", color="#059669")
    axes[2].legend(loc="upper right")
    axes[2].grid(True, alpha=0.3)

    fig.tight_layout()
    fig.savefig(str(PLOT_PATH), dpi=150)
    plt.close(fig)
    print(f"Zapisano wykresy: {PLOT_PATH}")


def run_interactive_viewer() -> None:
    print(f"[MuJoCo {mujoco.__version__}] Uruchamianie interaktywnego okna MuJoCo Viewer...")
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.ctrl[0] = DRIVE_TORQUE
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.lookat[:] = [0.0, 0.09, 0.0]
        viewer.cam.distance = 0.50
        viewer.cam.azimuth = -55.0
        viewer.cam.elevation = -14.0

        while viewer.is_running():
            step_start = time.perf_counter()
            data.ctrl[0] = DRIVE_TORQUE
            mujoco.mj_step(model, data)
            viewer.sync()
            elapsed = time.perf_counter() - step_start
            if elapsed < model.opt.timestep:
                time.sleep(model.opt.timestep - elapsed)


def main() -> None:
    parser = argparse.ArgumentParser(description="Symulacja mechanizmu rzeźby kinetycznej Theisolab w MuJoCo 4")
    parser.add_argument("--viewer", action="store_true", help="Uruchom interaktywne okno MuJoCo Viewer")
    parser.add_argument("--duration", type=float, default=10.0, help="Czas nagrania wideo [s]")
    args = parser.parse_args()

    if args.viewer:
        run_interactive_viewer()
    else:
        run_headless_recording(duration=args.duration)


if __name__ == "__main__":
    main()
