"""URDF model dict + GUI config  ->  MJCF XML (MuJoCo 4.x XML reference).

The *_FIELDS schemas below are the single source of truth: the GUI renders its forms from them
(sent via /api/load) and the generator maps the same keys onto MJCF attributes.
Field tuple: (key, label, type, extra) with type in
  num | int | vec<N> | sel | tri (inherit/true/false) | bool | text | multi
"""
import math
import os
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET

from urdf_parser import rpy_to_quat

MATERIALS = {  # kg/m^3
    "Aluminium 6061": 2700, "Aluminium 7075": 2810, "Steel": 7850, "Stainless steel": 8000,
    "Titanium": 4500, "Brass": 8500, "Copper": 8960, "Magnesium": 1740, "ABS": 1050, "PLA": 1240,
    "PETG": 1270, "Nylon (PA12)": 1010, "Polycarbonate": 1200, "POM (Delrin)": 1410,
    "Carbon fibre": 1600, "Fibreglass": 1850, "Wood (birch ply)": 680, "Rubber": 1100,
    "TPU": 1210, "Water": 1000,
}

# ---------------------------------------------------------------- schemas
def F(k, label, t="num", **x):
    return {"k": k, "label": label, "t": t, **x}

OPTION_FIELDS = [
    F("timestep", "Timestep (s)", hint="Most important speed/accuracy trade-off"),
    F("gravity", "Gravity", "vec3"),
    F("integrator", "Integrator", "sel", opts=["Euler", "RK4", "implicit", "implicitfast", "discrete"],
      hint="implicitfast recommended when using kv / dampratio / velocity servos"),
    F("cone", "Friction cone", "sel", opts=["pyramidal", "elliptic"]),
    F("solver", "Solver", "sel", opts=["Newton", "CG", "PGS"]),
    F("iterations", "Solver iterations", "int"),
    F("tolerance", "Solver tolerance"),
    F("ls_iterations", "Linesearch iterations", "int"),
    F("noslip_iterations", "NoSlip iterations", "int", hint=">0 suppresses slow slippage (e.g. feet)"),
    F("impratio", "Impedance ratio", hint=">1 makes friction 'harder' (elliptic cones)"),
    F("ccd_iterations", "Convex collision iters", "int"),
    F("wind", "Wind velocity", "vec3"),
    F("magnetic", "Magnetic field", "vec3", hint="Used by magnetometer sensors"),
    F("density", "Medium density", hint="Air 1.2, water 1000 (lift/drag)"),
    F("viscosity", "Medium viscosity", hint="Air 2e-5, water 9e-4"),
]
FLAGS = {  # name: default
    "constraint": "enable", "equality": "enable", "frictionloss": "enable", "limit": "enable",
    "contact": "enable", "spring": "enable", "damper": "enable", "gravity": "enable",
    "clampctrl": "enable", "warmstart": "enable", "filterparent": "enable", "actuation": "enable",
    "refsafe": "enable", "sensor": "enable", "midphase": "enable", "nativeccd": "enable",
    "eulerdamp": "enable", "autoreset": "enable", "multiccd": "enable", "island": "enable",
    "override": "disable", "energy": "disable", "fwdinv": "disable", "sleep": "disable",
    "diagexact": "disable", "speculative": "disable",
}
COMPILER_FIELDS = [
    F("balanceinertia", "Balance inertia", "bool", hint="Fix inertias violating A+B>=C"),
    F("boundmass", "Min body mass", hint="Quick fix for massless moving bodies"),
    F("boundinertia", "Min body inertia"),
    F("alignfree", "Align free bodies", "bool", hint="Align free joint with inertial frame (faster/more stable)"),
    F("fitaabb", "Fit primitives to AABB", "bool", hint="Otherwise fit to equivalent-inertia box"),
    F("discardvisual", "Discard visual geoms", "bool", hint="Smaller/faster model, keeps identical dynamics"),
    F("inertiagrouprange", "Inertia geom groups", "vec2"),
]
GEOM_FIELDS = [
    F("friction", "Friction (slide torsion roll)", "vec3"),
    F("condim", "Contact dims", "sel", opts=["", "1", "3", "4", "6"],
      hint="1 frictionless, 3 sliding, 4 +torsional, 6 +rolling"),
    F("solref", "Contact solref (timeconst dampratio)", "vec2"),
    F("solimp", "Contact solimp", "vec5"),
    F("margin", "Contact margin"),
    F("gap", "Contact gap"),
    F("priority", "Priority", "int"),
    F("solmix", "Solmix"),
]
JOINT_FIELDS = [
    F("damping", "Damping", hint="Viscous, N·m·s/rad"),
    F("frictionloss", "Friction loss", hint="Dry friction, N·m"),
    F("armature", "Armature", hint="Reflected rotor inertia (rotor × gear²); improves stability"),
    F("stiffness", "Spring stiffness"),
    F("springref", "Spring rest position"),
    F("ref", "Reference position", hint="Joint value at the model's initial configuration"),
    F("margin", "Limit margin"),
    F("solreflimit", "Limit solref", "vec2", hint="Soft limits: time constant, damping ratio"),
    F("solimplimit", "Limit solimp", "vec5"),
    F("solreffriction", "Friction solref", "vec2"),
    F("torqueLimit", "Joint torque clamp ±", hint="actuatorfrcrange: clamps total actuator force at the joint"),
    F("actuatorgravcomp", "Gravcomp through actuator", "tri"),
    F("group", "Group", "int"),
]
ACT_TYPES = ["none", "motor", "position", "velocity", "intvelocity", "pid", "damper", "dcmotor"]
ACT_DESC = {
    "none": "No actuator", "motor": "Direct torque command", "position": "PD position servo",
    "velocity": "Velocity servo", "intvelocity": "Integrated-velocity servo",
    "pid": "PID with pos/vel/ff inputs", "damper": "Active damper (F = -kv·v·ctrl)",
    "dcmotor": "DC motor model (electrical, thermal, cogging, friction)",
}
_ALL = set(ACT_TYPES) - {"none"}
ACT_FIELDS = [
    F("gear", "Gear ratio", types=_ALL),
    F("torqueLimit", "Torque limit ± (forcerange)", types=_ALL - {"dcmotor"}, hint="Clamp actuator output force"),
    F("ctrlrange", "Control range", "vec2", types=_ALL,
      hint="Blank: motor ±torque/gear, position = joint range, damper 0..1"),
    F("kp", "kp", types={"position", "intvelocity", "pid"}),
    F("kv", "kv", types={"position", "velocity", "intvelocity", "pid", "damper"}),
    F("dampratio", "Damping ratio", types={"position", "intvelocity", "pid"}, hint="1 = critically damped; replaces kv"),
    F("timeconst", "Filter time const", types={"position"}),
    F("inheritrange", "Inherit joint range ×", types={"position", "intvelocity", "pid"},
      hint="1.0 = control range equals joint range"),
    F("actrange", "Activation range", "vec2", types={"intvelocity"}),
    F("ki", "ki", types={"pid"}), F("imax", "Integral limit", types={"pid"}),
    F("slewmax", "Max setpoint rate", types={"pid"}),
    F("input", "Inputs", "text", types={"pid", "dcmotor"}, hint="pid: 'pos vel ff' subset · dcmotor: 'voltage' or 'pos vel ff'"),
    F("velrange", "Velocity input range", "vec2", types={"pid"}),
    F("ffrange", "Feedforward range", "vec2", types={"pid"}),
    F("resistance", "Resistance Ω", types={"dcmotor"}),
    F("motorconst", "Motor const Kt Ke", "vec2", types={"dcmotor"}),
    F("nominal", "Nominal V, stall τ, no-load ω", "vec3", types={"dcmotor"}),
    F("inductance", "Inductance L, timeconst", "vec2", types={"dcmotor"}),
    F("saturation", "Saturation τ, I, dI/dt", "vec3", types={"dcmotor"}),
    F("controller", "Controller kp ki kd slew Imax Vmax", "vec6", types={"dcmotor"}),
    F("cogging", "Cogging amp poles phase", "vec3", types={"dcmotor"}),
    F("lugre", "LuGre σ0 σ1 Fc Fs vs", "vec5", types={"dcmotor"}),
    F("thermal", "Thermal R C τ α Tref Tamb", "vec6", types={"dcmotor"}),
    F("armature", "Rotor inertia", types=_ALL, hint="Scaled by gear² at the joint"),
    F("damping", "Actuator damping", types=_ALL, hint="Scaled by gear² at the joint"),
    F("delay", "Control delay (s)", types=_ALL),
    F("nsample", "History samples", "int", types=_ALL, hint="Auto-set if delay is given"),
    F("group", "Group", "int", types=_ALL),
]
SENSOR_COMMON = [
    F("noise", "Noise σ"), F("cutoff", "Cutoff (|value| clip)"),
    F("nsample", "History samples", "int"), F("delay", "Delay (s)"),
    F("interval", "Sample period, phase", "vec2", hint="Lower sample rate than the sim (needs history)"),
]
# type -> (kind, description)
SENSORS = {
    "accelerometer": ("site", "Linear acceleration incl. gravity (site frame)"),
    "gyro": ("site", "Angular velocity (site frame)"),
    "velocimeter": ("site", "Linear velocity (site frame)"),
    "magnetometer": ("site", "Magnetic flux (site frame)"),
    "force": ("site", "3-axis force child→parent at site (F/T sensor)"),
    "torque": ("site", "3-axis torque child→parent at site (F/T sensor)"),
    "touch": ("site", "Normal contact force inside site volume"),
    "rangefinder": ("site", "Distance along site +Z axis"),
    "framepos": ("frame", "Position (world or reference frame)"),
    "framequat": ("frame", "Orientation quaternion"),
    "framexaxis": ("frame", "Frame X axis"), "frameyaxis": ("frame", "Frame Y axis"),
    "framezaxis": ("frame", "Frame Z axis"),
    "framelinvel": ("frame", "Linear velocity"), "frameangvel": ("frame", "Angular velocity"),
    "framelinacc": ("frame", "Linear acceleration"), "frameangacc": ("frame", "Angular acceleration"),
    "jointpos": ("joint", "Joint position"), "jointvel": ("joint", "Joint velocity"),
    "jointlimitpos": ("joint", "Limit violation (position)"),
    "jointlimitvel": ("joint", "Limit violation (velocity)"),
    "jointlimitfrc": ("joint", "Limit constraint force"),
    "jointactuatorfrc": ("joint", "Total actuator force at joint"),
    "actuatorpos": ("actuator", "Actuator length"), "actuatorvel": ("actuator", "Actuator velocity"),
    "actuatorfrc": ("actuator", "Actuator scalar force"),
    "subtreecom": ("body", "Subtree centre of mass"), "subtreelinvel": ("body", "Subtree linear velocity"),
    "subtreeangmom": ("body", "Subtree angular momentum"),
    "contact": ("contact", "Fixed-size contact report (feet, grippers)"),
    "clock": ("none", "Simulation time"), "e_potential": ("none", "Potential energy"),
    "e_kinetic": ("none", "Kinetic energy"),
}
CONTACT_DATA = ["found", "force", "torque", "dist", "pos", "normal", "tangent", "linvel", "angvel"]
VEC_DEFAULTS = {
    "friction": [1, 0.005, 0.0001], "solref": [0.02, 1], "solimp": [0.9, 0.95, 0.001, 0.5, 2],
    "solreflimit": [0.02, 1], "solimplimit": [0.9, 0.95, 0.001, 0.5, 2], "solreffriction": [0.02, 1],
    "gravity": [0, 0, -9.81], "wind": [0, 0, 0], "magnetic": [0, -0.5, 0], "interval": [0, 0],
    "rgba": [0.5, 0.5, 0.5, 1], "inertiagrouprange": [0, 5],
}


def schema_payload():
    def ser(fs):
        return [{**f, "types": sorted(f["types"])} if "types" in f else f for f in fs]
    return {"option": ser(OPTION_FIELDS), "flags": FLAGS, "compiler": ser(COMPILER_FIELDS),
            "geom": ser(GEOM_FIELDS), "joint": ser(JOINT_FIELDS), "actuator": ser(ACT_FIELDS),
            "actTypes": ACT_TYPES, "actDesc": ACT_DESC, "sensorCommon": ser(SENSOR_COMMON),
            "sensors": {k: {"kind": v[0], "desc": v[1]} for k, v in SENSORS.items()},
            "contactData": CONTACT_DATA, "materials": MATERIALS}


# ---------------------------------------------------------------- helpers
def _s(v):
    if isinstance(v, (list, tuple)):
        return " ".join(_s(x) for x in v)
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, float):
        return f"{v:.9g}"
    return str(v)


def _el(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {k: _s(v) for k, v in attrs.items() if v is not None})


def _clean(name):
    return re.sub(r"[^A-Za-z0-9_\-.]", "_", str(name))


def _num(x, default=None):
    try:
        if x is None or x == "" or isinstance(x, bool):
            return default
        return float(x)
    except (TypeError, ValueError):
        return default


def _blank(v):
    if v is None or v == "":
        return True
    if isinstance(v, list):
        return all(x is None or x == "" for x in v)
    return False


def _val(field, v, key=None):
    """Normalise a config value for the given field; None if unset."""
    if _blank(v):
        return None
    t = field["t"]
    if t.startswith("vec"):
        n = int(t[3:])
        dflt = VEC_DEFAULTS.get(key or field["k"], [0] * n)
        vv = list(v) + [None] * n
        return [(_num(vv[i]) if _num(vv[i]) is not None else (dflt[i] if i < len(dflt) else 0)) for i in range(n)]
    if t == "int":
        return int(float(v))
    if t == "num":
        return _num(v)
    if t in ("tri", "bool"):
        if isinstance(v, bool):
            return v
        return {"true": True, "false": False}.get(str(v))
    return v


def default_config(model):
    return {
        "global": {
            "modelName": model["name"], "meshScale": 1.0, "rootFree": False, "rootHeight": 0.0,
            "fusestatic": False, "floor": True, "floorSize": 5.0, "floorFriction": [1, 0.005, 0.0001],
            "prettyScene": True, "selfCollision": True, "showCollision": False,
            "option": {"timestep": 0.002, "gravity": [0, 0, -9.81], "integrator": "implicitfast"},
            "flags": {},
            "compiler": {"balanceinertia": True},
            "defaultMassMode": "urdf", "defaultMassScale": 1.0, "defaultMaterial": "Aluminium 6061",
            "defaultDensity": 2700, "defaultMeshInertia": "convex",
            "defaultCollision": "visual", "defaultFitscale": 1.0,
            "geomDefaults": {"friction": [1, 0.005, 0.0001]},
            "jointDefaults": {"armature": 0.01, "damping": 0.1},
            "actuatorDefaults": {"type": "none", "gear": 1.0, "kp": 50.0, "kv": 2.0},
        },
        "links": {n: {"massMode": "inherit", "geom": {}} for n in model["links"]},
        "joints": {n: {"limited": None, "lower": None, "upper": None, "dyn": {}, "actuator": {"type": "inherit"}}
                   for n in model["joints"]},
        "sensors": [], "cameras": [], "excludes": [], "keyframes": [],
    }


SCALAR_JOINTS = ("revolute", "continuous", "prismatic")


# ---------------------------------------------------------------- generator
class Gen:
    def __init__(self, model, cfg):
        self.m, self.cfg = model, cfg
        self.g = cfg.get("global", {})
        self.warnings = []
        self.meshes = {}        # key -> (name, scale, inertia)
        self.mesh_files = {}
        self.sites = {}
        self.site_by_body = {}
        self.joint_names = {}
        self.qpos_layout = []   # (kind, urdf joint name)
        self.act_order = []     # (urdf joint, act type)
        self.actuator_of = {}
        self.body_els = {}

    def gv(self, k, d=None):
        v = self.g.get(k, d)
        return d if v is None else v

    def lcfg(self, n):
        return self.cfg.get("links", {}).get(n, {}) or {}

    def jcfg(self, n):
        return self.cfg.get("joints", {}).get(n, {}) or {}

    def _inh(self, v, default):
        return default if v in (None, "", "inherit") else v

    def mass_mode(self, l):
        return self._inh(self.lcfg(l).get("massMode"), self.gv("defaultMassMode", "urdf"))

    def coll_mode(self, l):
        return self._inh(self.lcfg(l).get("collision"), self.gv("defaultCollision", "visual"))

    def mesh_inertia(self, l):
        return self._inh(self.lcfg(l).get("meshInertia"), self.gv("defaultMeshInertia", "convex"))

    def density_for(self, l):
        lc = self.lcfg(l)
        d = _num(lc.get("density"))
        if d:
            return d
        if lc.get("material") in MATERIALS:
            return float(MATERIALS[lc["material"]])
        return float(_num(self.g.get("defaultDensity"), 1000.0))

    # ---- meshes
    def mesh_asset(self, geom, inertia):
        path = geom.get("path")
        if not path:
            self.warnings.append(f"mesh not found: {geom.get('file')}")
            return None
        sc = [a * float(self.gv("meshScale", 1.0)) for a in geom.get("scale", [1, 1, 1])]
        key = (path, tuple(round(x, 9) for x in sc), inertia)
        if key not in self.meshes:
            base = _clean(os.path.splitext(os.path.basename(path))[0])
            if inertia != self.gv("defaultMeshInertia", "convex"):
                base += "_" + inertia
            name, i = base, 2
            while name in self.mesh_files:
                name, i = f"{base}_{i}", i + 1
            self.meshes[key] = (name, sc, inertia)
            self.mesh_files[name] = path
        return self.meshes[key]

    def add_geom(self, parent, g, cls, inertia, prim=None, **extra):
        at = {"class": cls, "pos": g["pos"] if any(g["pos"]) else None,
              "quat": rpy_to_quat(g["rpy"]) if any(abs(x) > 1e-12 for x in g["rpy"]) else None}
        t = g["type"]
        if t == "mesh":
            a = self.mesh_asset(g, inertia)
            if not a:
                return None
            at.update(type=prim or "mesh", mesh=a[0])
        elif t == "box":
            at.update(type="box", size=[x / 2 for x in g["size"]])
        elif t == "sphere":
            at.update(type="sphere", size=[g["radius"]])
        elif t == "cylinder":
            at.update(type="cylinder", size=[g["radius"], g["length"] / 2])
        else:
            return None
        if cls == "visual":
            rgba = self.lcfg(self._cur_link).get("rgba")
            if not _blank(rgba):
                at["rgba"] = _val({"t": "vec4"}, rgba, "rgba")
            elif "rgba" in g:
                at["rgba"] = g["rgba"]
        at.update({k: v for k, v in extra.items() if v is not None})
        return _el(parent, "geom", **at)

    def link_geoms(self, body, lname):
        self._cur_link = lname
        link, lc = self.m["links"][lname], self.lcfg(lname)
        mode, cmode, inert = self.mass_mode(lname), self.coll_mode(lname), self.mesh_inertia(lname)
        vis = link["visuals"]
        prim = None
        if cmode == "urdf":
            col = link["collisions"] or vis
        elif cmode == "visual":
            col = vis
        elif cmode.startswith("fit-"):
            col, prim = vis, cmode[4:]
        else:
            col = []
        if mode == "urdf":
            ine = link["inertial"]
            if ine and ine["mass"] > 0:
                s = _num(lc.get("massScale"), _num(self.g.get("defaultMassScale"), 1.0)) or 1.0
                i = [x * s for x in ine["inertia"]]
                at = {"pos": ine["pos"], "mass": ine["mass"] * s, "fullinertia": i}
                if any(abs(x) > 1e-12 for x in ine["rpy"]):
                    at["quat"] = rpy_to_quat(ine["rpy"])
                _el(body, "inertial", **at)
        vgeoms = [e for e in (self.add_geom(body, v, "visual", inert) for v in vis) if e is not None]
        gextra = {}
        for f in GEOM_FIELDS:
            v = _val(f, (lc.get("geom") or {}).get(f["k"]))
            if v is not None:
                gextra[f["k"]] = v
        if prim:
            fs = _num(lc.get("fitscale"), _num(self.g.get("defaultFitscale"), 1.0))
            if fs and fs != 1:
                gextra["fitscale"] = fs
        cgeoms = [e for e in (self.add_geom(body, c, "collision", inert, prim=prim, **gextra) for c in col)
                  if e is not None]
        if mode in ("density", "mass"):
            carriers = cgeoms or vgeoms
            if not carriers:
                if link["inertial"] and link["inertial"]["mass"] > 0:
                    self.warnings.append(f"link '{lname}' has no geometry to carry mass ({mode} mode)")
                return
            if mode == "density":
                for e in carriers:
                    e.set("density", _s(self.density_for(lname)))
            else:
                tot = _num(lc.get("mass"))
                if not tot or tot <= 0:
                    tot = link["inertial"]["mass"] if link["inertial"] else 1.0
                carriers[0].set("mass", _s(tot))
                for e in carriers[1:]:
                    e.set("density", "0")

    # ---- joints
    def joint_attrs(self, j):
        jc = self.jcfg(j["name"])
        dyn = jc.get("dyn") or {}
        at = {}
        ax = j["axis"]
        n = math.sqrt(sum(a * a for a in ax)) or 1.0
        at["type"] = "slide" if j["type"] == "prismatic" else "hinge"
        at["axis"] = [a / n for a in ax]
        if j["type"] in ("revolute", "prismatic"):
            lo = _num(jc.get("lower"), j["lower"])
            hi = _num(jc.get("upper"), j["upper"])
            lim = jc.get("limited")
            lim = (hi > lo) if lim in (None, "") else bool(lim)
            at["limited"] = lim
            if lim:
                at["range"] = [lo, hi]
        else:
            at["limited"] = False
        # URDF <dynamics> beats global default (emitted via <default>); per-joint override beats both
        if j["damping"]:
            at["damping"] = j["damping"]
        if j["friction"]:
            at["frictionloss"] = j["friction"]
        for f in JOINT_FIELDS:
            v = _val(f, dyn.get(f["k"]))
            if v is None and f["k"] == "torqueLimit":
                v = _val(f, (self.g.get("jointDefaults") or {}).get("torqueLimit"))
            if v is None:
                continue
            if f["k"] == "torqueLimit":
                if v > 0:
                    at["actuatorfrcrange"] = [-v, v]
            else:
                at[f["k"]] = v
        return at

    def build_body(self, parent_el, lname, joint, top=False):
        lc = self.lcfg(lname)
        at = {"name": _clean(lname)}
        if joint:
            if any(joint["pos"]):
                at["pos"] = joint["pos"]
            if any(abs(x) > 1e-12 for x in joint["rpy"]):
                at["quat"] = rpy_to_quat(joint["rpy"])
        gc = _num(lc.get("gravcomp"))
        if gc:
            at["gravcomp"] = gc
        body = _el(parent_el, "body", **at)
        if top and self.gv("rootFree", False):
            if self.gv("rootHeight", 0.0):
                body.set("pos", _s([0, 0, float(self.g["rootHeight"])]))
            _el(body, "freejoint", name=_clean(lname) + "_free")
            self.qpos_layout.append(("free", lname))
        if joint and joint["type"] in SCALAR_JOINTS:
            ja = self.joint_attrs(joint)
            ja["name"] = _clean(joint["name"])
            _el(body, "joint", **ja)
            self.joint_names[joint["name"]] = ja["name"]
            self.qpos_layout.append(("scalar", joint["name"]))
        elif joint and joint["type"] == "floating":
            _el(body, "freejoint", name=_clean(joint["name"]))
            self.qpos_layout.append(("free", joint["name"]))
        elif joint and joint["type"] == "planar":
            self.warnings.append(f"planar joint '{joint['name']}' approximated by two slide joints")
            for ax_, nm in (([1, 0, 0], "_x"), ([0, 1, 0], "_y")):
                _el(body, "joint", name=_clean(joint["name"]) + nm, type="slide", axis=ax_)
                self.qpos_layout.append(("planar", joint["name"]))
        self.link_geoms(body, lname)
        self.body_els[lname] = body
        for jn in self.m["children"].get(lname, []):
            cj = self.m["joints"][jn]
            self.build_body(body, cj["child"], cj)

    # ---- actuators
    def build_actuators(self):
        ad = self.gv("actuatorDefaults", {}) or {}
        act = ET.Element("actuator")
        ts = _num((self.g.get("option") or {}).get("timestep"), 0.002)
        for jn, j in self.m["joints"].items():
            if j["type"] not in SCALAR_JOINTS or jn not in self.joint_names:
                continue
            ac = self.jcfg(jn).get("actuator") or {}
            t = self._inh(ac.get("type"), ad.get("type", "none"))
            if t not in _ALL:
                continue
            vals = {}
            for f in ACT_FIELDS:
                if t not in f["types"]:
                    continue
                v = _val(f, ac.get(f["k"]))
                if v is None:
                    v = _val(f, ad.get(f["k"]))
                if v is not None:
                    vals[f["k"]] = v
            at = {"name": _clean(jn) + "_act", "joint": self.joint_names[jn]}
            gear = vals.pop("gear", 1.0) or 1.0
            if gear != 1.0:
                at["gear"] = gear
            tl = vals.pop("torqueLimit", None)
            if tl and tl > 0:
                at["forcerange"] = [-tl, tl]
            cr = vals.pop("ctrlrange", None)
            if vals.get("dampratio"):
                vals.pop("kv", None)
            if vals.get("inheritrange"):
                cr = None
            elif cr is None:
                jr = self.joint_attrs(j).get("range")
                if t == "motor" and tl and tl > 0:
                    cr = [-tl / gear, tl / gear]
                elif t in ("position", "pid") and jr:
                    cr = jr
                elif t == "damper":
                    cr = [0, 1]
            if cr is not None:
                at["ctrlrange"] = cr
            if vals.get("delay") and not vals.get("nsample"):
                vals["nsample"] = int(math.ceil(vals["delay"] / ts)) + 1
            if t == "dcmotor" and not any(k in vals for k in ("resistance", "motorconst", "nominal")):
                self.warnings.append(f"dcmotor on '{jn}' needs resistance/motorconst or nominal — skipped")
                continue
            at.update(vals)
            _el(act, t, **at)
            self.actuator_of[jn] = at["name"]
            ins = {"pid": "pos vel", "dcmotor": "voltage"}.get(t, "pos" if t == "position" else "u")
            ins = (vals.get("input") or ins).split()
            self.act_order.append((jn, t, [] if ins == ["none"] else ins))
        return act if len(act) else None

    # ---- sensors
    def site_for(self, body, pos, euler, stype, ssize, hint):
        key = (body, tuple(round(x, 9) for x in pos), tuple(round(x, 9) for x in euler), stype, tuple(ssize or ()))
        if key in self.sites:
            return self.sites[key]
        used = set(self.sites.values())
        name = base = _clean(f"{hint}_site")
        i = 2
        while name in used:
            name, i = f"{base}{i}", i + 1
        self.sites[key] = name
        self.site_by_body.setdefault(body, []).append((name, pos, euler, stype, ssize))
        return name

    def prepare_sensors(self):
        out, used = [], set()
        for s in self.cfg.get("sensors", []):
            t = s.get("type")
            if t not in SENSORS:
                continue
            kind = SENSORS[t][0]
            name = base = _clean(s.get("name") or t)
            i = 2
            while name in used:
                name, i = f"{base}_{i}", i + 1
            used.add(name)
            at = {"name": name}
            for f in SENSOR_COMMON:
                v = _val(f, s.get(f["k"]))
                if v is not None and v != 0 and v != [0, 0]:
                    at[f["k"]] = v
            if (at.get("delay") or at.get("interval")) and not at.get("nsample"):
                at["nsample"] = 10
            if kind in ("site", "frame"):
                body = s.get("body")
                if body not in self.m["links"]:
                    self.warnings.append(f"sensor '{name}': unknown body '{body}'")
                    continue
                pos = [_num(x, 0.0) for x in (s.get("pos") or [0, 0, 0])][:3]
                eul = [_num(x, 0.0) for x in (s.get("euler") or [0, 0, 0])][:3]
                stype = s.get("siteType") or "sphere"
                ssize = None if _blank(s.get("siteSize")) else [_num(x, 0.01) for x in s["siteSize"]][:3]
                site = self.site_for(body, pos, eul, stype, ssize, name)
                if kind == "frame":
                    at.update(objtype="site", objname=site)
                    rb = s.get("refBody")
                    if rb and rb in self.m["links"]:
                        at.update(reftype="xbody", refname=_clean(rb))
                else:
                    at["site"] = site
            elif kind == "joint":
                if s.get("joint") not in self.joint_names:
                    self.warnings.append(f"sensor '{name}': joint '{s.get('joint')}' has no MJCF joint")
                    continue
                at["joint"] = self.joint_names[s["joint"]]
            elif kind == "actuator":
                if s.get("joint") not in self.actuator_of:
                    self.warnings.append(f"sensor '{name}': joint '{s.get('joint')}' has no actuator")
                    continue
                at["actuator"] = self.actuator_of[s["joint"]]
            elif kind == "body":
                if s.get("body") not in self.m["links"]:
                    continue
                at["body"] = _clean(s["body"])
            elif kind == "contact":
                b1 = s.get("body")
                if b1 not in self.m["links"]:
                    self.warnings.append(f"contact sensor '{name}' needs a body")
                    continue
                at["subtree1" if s.get("subtree") else "body1"] = _clean(b1)
                o = s.get("other") or ""
                if o == "floor":
                    if self.gv("floor", True):
                        at["geom2"] = "floor"
                elif o in self.m["links"]:
                    at["body2"] = _clean(o)
                data = [d for d in CONTACT_DATA if d in (s.get("data") or ["found", "force"])]
                at["data"] = " ".join(data) or "found"
                at["num"] = int(_num(s.get("num"), 1) or 1)
                if s.get("reduce") and s["reduce"] != "none":
                    at["reduce"] = s["reduce"]
            out.append((t, at))
        return out

    # ---- keyframes
    def keyframes(self, root):
        kfs = self.cfg.get("keyframes") or []
        if not kfs:
            return
        kel = ET.SubElement(root, "keyframe")
        h = float(self.gv("rootHeight", 0.0) or 0.0)
        for k in kfs:
            q = k.get("qpos") or {}
            qpos = []
            for kind, jn in self.qpos_layout:
                if kind == "free":
                    qpos += [0, 0, h, 1, 0, 0, 0]
                else:
                    qpos.append(_num(q.get(jn), 0.0))
            at = {"name": _clean(k.get("name") or "key"), "qpos": qpos}
            if k.get("ctrlFromPose") and self.act_order:
                at["ctrl"] = [(_num(q.get(jn), 0.0) if tok == "pos" else 0.0)
                              for jn, t, ins in self.act_order for tok in ins]
            _el(kel, "key", **at)

    # ---- main
    def generate(self, meshdir):
        m, g = self.m, self.g
        root = ET.Element("mujoco", model=_clean(self.gv("modelName", m["name"])))
        comp = {"angle": "radian", "meshdir": meshdir, "autolimits": "true",
                "fusestatic": bool(self.gv("fusestatic", False)) or bool(self.gv("rootFree", False))}
        for f in COMPILER_FIELDS:
            v = _val(f, (g.get("compiler") or {}).get(f["k"]))
            if v is not None and v is not False and v != 0:
                comp[f["k"]] = v
        _el(root, "compiler", **comp)
        opt = {}
        for f in OPTION_FIELDS:
            v = _val(f, (g.get("option") or {}).get(f["k"]))
            if v is not None:
                opt[f["k"]] = v
        oel = _el(root, "option", **opt)
        flags = {k: v for k, v in (g.get("flags") or {}).items() if k in FLAGS and v in ("enable", "disable") and v != FLAGS[k]}
        if flags:
            _el(oel, "flag", **flags)
        sc = bool(self.gv("selfCollision", True))
        d = _el(root, "default")
        jd = {}
        for f in JOINT_FIELDS:
            if f["k"] == "torqueLimit":
                continue
            v = _val(f, (g.get("jointDefaults") or {}).get(f["k"]))
            if v is not None:
                jd[f["k"]] = v
        if jd:
            _el(d, "joint", **jd)
        _el(d, "site", size=0.008, rgba=[1, 0.5, 0, 1], group=4)
        dv = _el(d, "default", **{"class": "visual"})
        _el(dv, "geom", contype=0, conaffinity=0, group=2, density=0)
        dc = _el(d, "default", **{"class": "collision"})
        gd = {}
        for f in GEOM_FIELDS:
            v = _val(f, (g.get("geomDefaults") or {}).get(f["k"]))
            if v is not None:
                gd[f["k"]] = v
        _el(dc, "geom", group=1 if self.gv("showCollision", False) else 3, contype=1,
            conaffinity=1 if sc else 0, rgba=[0.8, 0.3, 0.3, 0.35], **gd)
        top = m["roots"][0] if m["roots"] else None
        if top is None:
            raise ValueError("URDF has no root link")
        wb = ET.Element("worldbody")
        _el(wb, "light", pos=[0, 0, 3], dir=[0, 0, -1], diffuse=[0.8, 0.8, 0.8], directional=True)
        pretty = bool(self.gv("prettyScene", True))
        if self.gv("floor", True):
            fs = _num(g.get("floorSize"), 5.0) or 5.0
            _el(wb, "geom", name="floor", type="plane", size=[fs, fs, 0.05],
                material="groundplane" if pretty else None, rgba=None if pretty else [0.35, 0.4, 0.45, 1],
                friction=_val({"t": "vec3"}, g.get("floorFriction"), "friction"),
                contype=1 if sc else 0, conaffinity=1, group=0)
        for r in m["roots"]:
            self.build_body(wb, r, None, top=(r == top))
        actuators = self.build_actuators()
        sensors = self.prepare_sensors()
        for body, lst in self.site_by_body.items():
            be = self.body_els[body]
            for name, pos, eul, stype, ssize in lst:
                at = {"name": name, "pos": pos if any(pos) else None,
                      "quat": rpy_to_quat(eul) if any(eul) else None,
                      "type": stype if stype != "sphere" else None, "size": ssize}
                _el(be, "site", **at)
        for c in self.cfg.get("cameras") or []:
            b = c.get("body") or ""
            parent = self.body_els.get(b, wb)
            at = {"name": _clean(c.get("name") or "cam"),
                  "pos": [_num(x, 0.0) for x in (c.get("pos") or [0, 0, 0])][:3],
                  "mode": c.get("mode") if c.get("mode") not in (None, "", "fixed") else None,
                  "fovy": _num(c.get("fovy"))}
            eul = [_num(x, 0.0) for x in (c.get("euler") or [0, 0, 0])][:3]
            if any(eul):
                at["quat"] = rpy_to_quat(eul)
            if at["mode"] in ("targetbody", "targetbodycom"):
                tgt = c.get("target")
                if tgt not in self.m["links"]:
                    self.warnings.append(f"camera '{at['name']}' needs a target body")
                    at["mode"] = None
                else:
                    at["target"] = _clean(tgt)
            _el(parent, "camera", **at)
        asset = ET.SubElement(root, "asset")
        if pretty:
            _el(asset, "texture", type="skybox", builtin="gradient", rgb1=[0.3, 0.5, 0.7], rgb2=[0, 0, 0],
                width=512, height=3072)
            _el(asset, "texture", type="2d", name="groundplane", builtin="checker", mark="edge",
                rgb1=[0.2, 0.3, 0.4], rgb2=[0.1, 0.2, 0.3], markrgb=[0.8, 0.8, 0.8], width=300, height=300)
            _el(asset, "material", name="groundplane", texture="groundplane", texuniform=True,
                texrepeat=[5, 5], reflectance=0.2)
        for (path, _, _), (name, scale, inertia) in self.meshes.items():
            _el(asset, "mesh", name=name, file=self.rel_mesh(path), inertia=inertia,
                scale=scale if any(abs(x - 1) > 1e-12 for x in scale) else None)
        if not len(asset):
            root.remove(asset)
        root.append(wb)
        ex = [e for e in (self.cfg.get("excludes") or []) if e.get("b1") in m["links"] and e.get("b2") in m["links"]]
        if ex:
            ce = ET.SubElement(root, "contact")
            for e in ex:
                _el(ce, "exclude", body1=_clean(e["b1"]), body2=_clean(e["b2"]))
        if actuators is not None:
            root.append(actuators)
        eq = None
        for jn, j in m["joints"].items():
            mi = j.get("mimic")
            if mi and jn in self.joint_names and mi["joint"] in self.joint_names:
                eq = eq if eq is not None else ET.SubElement(root, "equality")
                _el(eq, "joint", joint1=self.joint_names[jn], joint2=self.joint_names[mi["joint"]],
                    polycoef=[mi["offset"], mi["multiplier"], 0, 0, 0])
        if sensors:
            se = ET.SubElement(root, "sensor")
            for t, at in sensors:
                _el(se, t, **at)
        self.keyframes(root)
        ET.indent(root, space="  ")
        return ET.tostring(root, encoding="unicode")

    def rel_mesh(self, path):
        return os.path.relpath(path, self.mesh_root).replace(os.sep, "/")

    def run(self, meshdir_mode):
        files = [g["path"] for l in self.m["links"].values() for g in l["visuals"] + l["collisions"] if g.get("path")]
        self.mesh_root = os.path.commonpath([os.path.dirname(f) for f in files]) if files else self.m["urdf_dir"]
        return self.generate(self.mesh_root if meshdir_mode == "absolute" else "meshes")


def generate_mjcf(model, cfg, mode="absolute"):
    gen = Gen(model, cfg)
    return gen.run(mode), gen


def validate(xml):
    """Compile with MuJoCo (if installed) and run a short simulation."""
    try:
        import mujoco
    except ImportError:
        return {"ok": None, "message": "mujoco python package not installed - skipped compile check"}
    d = tempfile.mkdtemp(prefix="mjcf_")
    try:
        p = os.path.join(d, "m.xml")
        with open(p, "w") as f:
            f.write(xml)
        try:
            m = mujoco.MjModel.from_xml_path(p)
        except Exception as e:  # noqa
            return {"ok": False, "message": str(e)}
        data = mujoco.MjData(m)
        if m.nkey:
            mujoco.mj_resetDataKeyframe(m, data, 0)
        stable = True
        try:
            for _ in range(250):
                mujoco.mj_step(m, data)
            stable = all(math.isfinite(x) for x in data.qpos)
        except Exception:  # noqa
            stable = False
        masses = {}
        for i in range(1, m.nbody):
            masses[mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) or f"body{i}"] = float(m.body_mass[i])
        sensors = []
        for i in range(m.nsensor):
            sensors.append({"name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SENSOR, i), "dim": int(m.sensor_dim[i])})
        return {"ok": True, "version": mujoco.__version__, "nbody": int(m.nbody), "njnt": int(m.njnt),
                "nu": int(m.nu), "na": int(m.na), "nsensor": int(m.nsensor), "nsensordata": int(m.nsensordata),
                "ngeom": int(m.ngeom), "nq": int(m.nq), "nv": int(m.nv), "nkey": int(m.nkey),
                "totalMass": float(sum(m.body_mass)), "masses": masses, "sensors": sensors,
                "stable": stable, "message": "compiled OK"}
    finally:
        shutil.rmtree(d, ignore_errors=True)


def save_package(model, cfg, out_dir, filename):
    gen = Gen(model, cfg)
    xml = gen.run("relative")
    os.makedirs(os.path.join(out_dir, "meshes"), exist_ok=True)
    for (path, _, _), _v in gen.meshes.items():
        dst = os.path.join(out_dir, "meshes", gen.rel_mesh(path))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        shutil.copy2(path, dst)
    p = os.path.join(out_dir, filename)
    with open(p, "w") as f:
        f.write(xml)
    return p, gen.warnings
