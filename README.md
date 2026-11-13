# Modeling

Tools for turning robot descriptions into MuJoCo models.

## URDF → MJCF Studio (`urdf2mjcf/`)

An interactive, browser-based GUI that loads a URDF folder, converts it to MuJoCo's MJCF format, and lets you
configure the physics, motors and sensors by hand before exporting. Targets the MuJoCo 4.x
[XML reference](https://mujoco.readthedocs.io/en/stable/XMLreference.html).

### Quick start

```bash
python -m venv .venv
.venv/bin/pip install mujoco          # optional: enables live compile + stability check
.venv/bin/python urdf2mjcf/server.py CENTAUR    # opens http://127.0.0.1:8765
```

Pass any URDF file or a folder containing one, or run it with no argument and use **Browse…** in the GUI.
Options: `--port N`, `--no-browser`. The backend uses only the Python standard library; `mujoco` is only needed
for validation. The 3D preview loads three.js from a CDN (needs internet); everything else works offline.

### What you can configure

| Tab | Contents |
|---|---|
| **Model** | Floating base and start height, mesh scale, floor, self-collision, compiler options |
| **Physics** | `<option>` (timestep, integrator, solver, cone, wind, medium) and all pipeline flags |
| **Defaults** | Mass source (URDF inertia × scale, uniform density from 20 material presets, or fixed total mass), mesh volume method, collision geometry (visual / URDF / fitted primitives), contact parameters, joint dynamics, actuators for all joints, URDF-vs-MJCF mass report |
| **Selected** | Per-link and per-joint overrides (inherited values shown as placeholders), joint pose preview, copy settings to joints matching a regex, quick sensors |
| **Sensors** | All sensor types with mount offset/rotation, noise, cutoff, delay, sample rate; one-click IMU, encoders, joint torques, foot contacts |
| **Extras** | Cameras, contact excludes, keyframes (captured from the preview pose), sensordata layout |

**Actuators:** motor, position, velocity, intvelocity, pid, damper and dcmotor (resistance / motor constants /
nominal point, inductance, saturation, controller, cogging, LuGre friction, thermal), plus gear, torque limit,
control range, rotor inertia, damping and control delay.

### Output

`Save MJCF` writes to the chosen folder:

```
<out>/<name>.xml                 the MJCF model (relative meshdir)
<out>/meshes/                    mesh files copied from the URDF package
<out>/urdf2mjcf_config.json      every GUI setting, re-importable via "Import config"
```

### Notes

- **Uniform density vs. CAD mass:** CAD parts are often hollow, so a solid-material density can be far heavier
  than the URDF's inertial values (on the included CENTAUR model: ~194 kg solid aluminium vs. 58 kg in the URDF).
  The mass report in the Defaults tab compares them per link.
- Mesh volume method `exact` needs watertight meshes; the default is `convex`.
- Mimic joints become equality constraints; planar joints are approximated by two slide joints.
- MJCF has no joint velocity limit, so a URDF velocity limit only sets the control range of velocity servos.

### Layout

```
urdf2mjcf/
  server.py        HTTP server + API (load / generate / save)
  urdf_parser.py   dependency-free URDF parser
  mjcf_gen.py      config → MJCF generator; field schemas that also drive the GUI forms
  index.html       single-page GUI
CENTAUR/           example robot (SolidWorks-exported URDF + STL meshes)
```

To expose a new MJCF attribute, add it to the relevant schema list at the top of `mjcf_gen.py`; the form appears in
the GUI automatically.
