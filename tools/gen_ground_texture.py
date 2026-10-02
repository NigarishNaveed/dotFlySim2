#!/usr/bin/env python3
"""
gen_ground_texture.py -- generate models/ground_scrub/: a tiling scrub texture
and the ground mesh that repeats it. Run from the repo root:

    python3 tools/gen_ground_texture.py          # needs numpy and pillow

WHY THIS EXISTS
The ground used to be one flat colour. To a camera that is a surface with no
features at all, and a visual-inertial estimator (projects/vio_benchmark)
measures its own motion by tracking points that shift as it moves -- in dotFlySim,
with the same flat ground, OpenVINS found 0 trackable corners below the horizon
and drifted 9 m during a plain vertical climb. Real ground is the opposite:
grass, bushes, bare soil and shadow, at every scale.

WHAT IS GENERATED
  models/ground_scrub/materials/textures/scrub.png  2048 x 2048, seamless,
      covering TILE_M = 32 m: 1.6 cm per pixel. Detail at every scale a camera
      at 0-45 m sees: fine grain (cm), tufts and bushes (0.15-1.2 m), bare-soil
      patches (1-4 m), broad variation (up to the tile).
  models/ground_scrub/meshes/ground.obj (+ .mtl)  a 4 km x 4 km grid whose
      texture coordinates repeat the tile every 32 m. A Gazebo <plane> stretches
      one texture across the whole plane, so a mesh is what makes it tile.

COLOUR IS KEPT SATURATED, ON PURPOSE. demo_camera_track finds a lattice tower
by LOW saturation against the ground (see worlds/powerline.sdf), so only
brightness and the scrub/soil mix vary; nothing here is grey.

Deterministic (fixed seed): rerunning reproduces the same files.
"""
import os

import numpy as np
from PIL import Image

SIZE = 2048                # texture pixels per side
TILE_M = 32.0              # metres of ground one texture tile covers
EXTENT_M = 4000.0          # ground mesh side, matching the old visual plane
GRID = 8                   # mesh quads per side (keeps texture coords modest)
SEED = 20261002

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "ground_scrub")
PX_PER_M = SIZE / TILE_M

# Base colours, linear-ish 0..1. SCRUB is the old flat ground colour, so the
# world keeps its look from a distance.
SCRUB = np.array([0.38, 0.44, 0.22])
BUSH = np.array([0.17, 0.25, 0.08])
SOIL = np.array([0.50, 0.42, 0.26])


def periodic_noise(rng, beta, min_wavelength_m, max_wavelength_m):
    """Seamless fractal noise: white noise shaped by 1/f^beta in the FFT
    domain, band-limited to [min, max] wavelength. Anything built from an
    FFT of a periodic grid tiles perfectly."""
    white = rng.standard_normal((SIZE, SIZE))
    f = np.fft.fftfreq(SIZE) * PX_PER_M           # cycles per metre
    fx, fy = np.meshgrid(f, f)
    fr = np.hypot(fx, fy)
    band = (fr >= 1.0 / max_wavelength_m) & (fr <= 1.0 / min_wavelength_m)
    shape = np.zeros_like(fr)
    shape[band] = fr[band] ** (-beta)
    out = np.real(np.fft.ifft2(np.fft.fft2(white) * shape))
    out -= out.mean()
    return out / (out.std() + 1e-12)


def blobs(rng, count, r_min_m, r_max_m):
    """Soft discs placed with wrap-around, so the result still tiles."""
    acc = np.zeros((SIZE, SIZE))
    yy, xx = np.mgrid[0:SIZE, 0:SIZE]
    for _ in range(count):
        r = rng.uniform(r_min_m, r_max_m) * PX_PER_M
        cx, cy = rng.uniform(0, SIZE, 2)
        dx = np.abs(xx - cx); dx = np.minimum(dx, SIZE - dx)
        dy = np.abs(yy - cy); dy = np.minimum(dy, SIZE - dy)
        # cheap reject: only touch the bounding square
        if r < 1:
            continue
        d = np.hypot(dx, dy) / r
        acc = np.maximum(acc, np.clip(1.0 - d, 0.0, 1.0) ** 0.6)
    return acc


def make_texture():
    rng = np.random.default_rng(SEED)
    broad = periodic_noise(rng, 1.2, 2.0, TILE_M)        # metres-scale patchiness
    mid = periodic_noise(rng, 1.0, 0.15, 2.0)            # tufts
    fine = periodic_noise(rng, 0.6, 0.03, 0.15)          # grain
    bush = blobs(rng, 900, 0.15, 1.2)                    # scrub bushes
    soil = np.clip(blobs(rng, 70, 1.0, 4.0) * 1.3 + 0.25 * broad, 0.0, 1.0)

    # brightness: broad and mid variation plus grain, in roughly 0.55 .. 1.45
    light = 1.0 + 0.18 * broad + 0.16 * mid + 0.10 * fine
    light = np.clip(light, 0.55, 1.45)

    rgb = SCRUB[None, None, :] * np.ones((SIZE, SIZE, 1))
    rgb = rgb * (1 - soil[..., None] * 0.85) + SOIL[None, None, :] * (soil[..., None] * 0.85)
    rgb = rgb * (1 - bush[..., None]) + BUSH[None, None, :] * bush[..., None]
    rgb = rgb * light[..., None]
    # bush shadows: a little darker just beside each bush (sun from -x,+y,
    # as the world's directional light), shifted copy of the bush mask
    shadow = np.roll(np.roll(bush, 6, axis=1), -2, axis=0)
    rgb *= (1.0 - 0.25 * np.clip(shadow - bush, 0.0, 1.0))[..., None]
    rgb = np.clip(rgb, 0.0, 1.0)
    return (rgb ** (1 / 2.2) * 255.0 + 0.5).astype(np.uint8)   # sRGB-ish


def write_mesh():
    half = EXTENT_M / 2.0
    step = EXTENT_M / GRID
    v, vt, f = [], [], []
    for j in range(GRID + 1):
        for i in range(GRID + 1):
            x = -half + i * step
            y = -half + j * step
            v.append(f"v {x:.3f} {y:.3f} 0.000")
            vt.append(f"vt {x / TILE_M:.5f} {y / TILE_M:.5f}")
    for j in range(GRID):
        for i in range(GRID):
            a = j * (GRID + 1) + i + 1
            b, c, d = a + 1, a + GRID + 2, a + GRID + 1
            f.append(f"f {a}/{a}/1 {b}/{b}/1 {c}/{c}/1")
            f.append(f"f {a}/{a}/1 {c}/{c}/1 {d}/{d}/1")
    with open(os.path.join(ROOT, "meshes", "ground.obj"), "w") as fh:
        fh.write("# GENERATED by tools/gen_ground_texture.py -- texture repeats every "
                 f"{TILE_M:g} m\nmtllib ground.mtl\nusemtl scrub\n")
        fh.write("\n".join(v) + "\n" + "\n".join(vt) + "\nvn 0 0 1\n" + "\n".join(f) + "\n")
    with open(os.path.join(ROOT, "meshes", "ground.mtl"), "w") as fh:
        fh.write("# GENERATED by tools/gen_ground_texture.py\nnewmtl scrub\n"
                 "Ka 1 1 1\nKd 1 1 1\nKs 0.03 0.03 0.03\nNs 4\n"
                 "map_Kd ../materials/textures/scrub.png\n")


def write_model_files():
    with open(os.path.join(ROOT, "model.config"), "w") as fh:
        fh.write("""<?xml version="1.0"?>
<model>
  <name>ground_scrub</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>Tiling scrub-ground visual (generated by tools/gen_ground_texture.py).</description>
</model>
""")
    with open(os.path.join(ROOT, "model.sdf"), "w") as fh:
        fh.write("""<?xml version="1.0"?>
<!-- GENERATED by tools/gen_ground_texture.py. The world's ground_plane uses
     meshes/ground.obj directly as its visual; this model exists so the asset
     is a proper Gazebo model (and so walker lists it as scenery). -->
<sdf version="1.9">
  <model name="ground_scrub">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <cast_shadows>false</cast_shadows>
        <geometry><mesh><uri>model://ground_scrub/meshes/ground.obj</uri></mesh></geometry>
      </visual>
    </link>
  </model>
</sdf>
""")
    with open(os.path.join(ROOT, "walker.toml"), "w") as fh:
        fh.write('# Texture asset for the worlds\' ground plane; not a drone.\nkind = "scenery"\n')


def main():
    for sub in ("meshes", os.path.join("materials", "textures")):
        os.makedirs(os.path.join(ROOT, sub), exist_ok=True)
    Image.fromarray(make_texture(), "RGB").save(
        os.path.join(ROOT, "materials", "textures", "scrub.png"), optimize=True)
    write_mesh()
    write_model_files()
    print(f"wrote {os.path.normpath(ROOT)}  ({SIZE}px over {TILE_M:g} m = "
          f"{100 * TILE_M / SIZE:.2f} cm/px, mesh {EXTENT_M:g} m)")


if __name__ == "__main__":
    main()
