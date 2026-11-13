"""Minimal, dependency-free URDF parser producing a JSON-friendly model dict."""
import math
import os
import xml.etree.ElementTree as ET


def _floats(s, n, default=0.0):
    if s is None:
        return [default] * n
    v = [float(x) for x in s.split()]
    return (v + [default] * n)[:n]


def _origin(el):
    o = el.find("origin") if el is not None else None
    if o is None:
        return [0, 0, 0], [0, 0, 0]
    return _floats(o.get("xyz"), 3), _floats(o.get("rpy"), 3)


def rpy_to_quat(rpy):
    """URDF fixed-axis rpy (R = Rz(y) Ry(p) Rx(r)) -> quaternion (w, x, y, z)."""
    r, p, y = rpy
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ]


def resolve_mesh(filename, urdf_dir):
    """Resolve package:// / file:// / relative mesh path to an absolute file (or None)."""
    f = filename
    if f.startswith("file://"):
        f = f[7:]
    if f.startswith("package://"):
        rest = f[len("package://"):]
        pkg, _, rel = rest.partition("/")
        d = os.path.abspath(urdf_dir)
        while True:
            if os.path.basename(d) == pkg and os.path.isfile(os.path.join(d, rel)):
                return os.path.join(d, rel)
            if os.path.isfile(os.path.join(d, rel)):
                return os.path.join(d, rel)
            nd = os.path.dirname(d)
            if nd == d:
                break
            d = nd
        return None
    if os.path.isabs(f):
        return f if os.path.isfile(f) else None
    p = os.path.normpath(os.path.join(urdf_dir, f))
    if os.path.isfile(p):
        return p
    d = os.path.abspath(urdf_dir)
    while True:  # search upward
        p = os.path.join(d, f)
        if os.path.isfile(p):
            return p
        nd = os.path.dirname(d)
        if nd == d:
            return None
        d = nd


def _geom(el, urdf_dir):
    if el is None:
        return None
    pos, rpy = _origin(el)
    g = el.find("geometry")
    if g is None or len(g) == 0:
        return None
    s = g[0]
    out = {"pos": pos, "rpy": rpy, "type": s.tag}
    if s.tag == "mesh":
        out["file"] = s.get("filename")
        out["path"] = resolve_mesh(s.get("filename"), urdf_dir)
        out["scale"] = _floats(s.get("scale"), 3, 1.0) if s.get("scale") else [1, 1, 1]
    elif s.tag == "box":
        out["size"] = _floats(s.get("size"), 3)
    elif s.tag == "sphere":
        out["radius"] = float(s.get("radius", 0))
    elif s.tag == "cylinder":
        out["radius"] = float(s.get("radius", 0))
        out["length"] = float(s.get("length", 0))
    m = el.find("material")
    if m is not None:
        c = m.find("color")
        if c is not None:
            out["rgba"] = _floats(c.get("rgba"), 4, 1.0)
    return out


def parse_urdf(path):
    path = os.path.abspath(path)
    urdf_dir = os.path.dirname(path)
    root = ET.parse(path).getroot()
    links, joints = {}, {}
    for l in root.findall("link"):
        name = l.get("name")
        d = {"name": name, "inertial": None, "visuals": [], "collisions": []}
        i = l.find("inertial")
        if i is not None:
            pos, rpy = _origin(i)
            ie = i.find("inertia")
            d["inertial"] = {
                "pos": pos, "rpy": rpy,
                "mass": float(i.find("mass").get("value")) if i.find("mass") is not None else 0.0,
                "inertia": [float(ie.get(k, 0)) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
                if ie is not None else [0] * 6,
            }
        for v in l.findall("visual"):
            g = _geom(v, urdf_dir)
            if g:
                d["visuals"].append(g)
        for c in l.findall("collision"):
            g = _geom(c, urdf_dir)
            if g:
                d["collisions"].append(g)
        links[name] = d
    for j in root.findall("joint"):
        name = j.get("name")
        pos, rpy = _origin(j)
        ax = j.find("axis")
        lim = j.find("limit")
        dyn = j.find("dynamics")
        mim = j.find("mimic")
        joints[name] = {
            "name": name, "type": j.get("type"),
            "parent": j.find("parent").get("link"), "child": j.find("child").get("link"),
            "pos": pos, "rpy": rpy,
            "axis": _floats(ax.get("xyz"), 3) if ax is not None else [1, 0, 0],
            "lower": float(lim.get("lower", 0)) if lim is not None else 0.0,
            "upper": float(lim.get("upper", 0)) if lim is not None else 0.0,
            "effort": float(lim.get("effort", 0)) if lim is not None else 0.0,
            "velocity": float(lim.get("velocity", 0)) if lim is not None else 0.0,
            "damping": float(dyn.get("damping", 0)) if dyn is not None else 0.0,
            "friction": float(dyn.get("friction", 0)) if dyn is not None else 0.0,
            "mimic": ({"joint": mim.get("joint"), "multiplier": float(mim.get("multiplier", 1)),
                       "offset": float(mim.get("offset", 0))} if mim is not None else None),
        }
    children = {j["parent"]: [] for j in joints.values()}
    for j in joints.values():
        children.setdefault(j["parent"], []).append(j["name"])
    child_links = {j["child"] for j in joints.values()}
    roots = [n for n in links if n not in child_links]
    return {
        "name": root.get("name", "robot"), "path": path, "urdf_dir": urdf_dir,
        "links": links, "joints": joints, "children": children, "roots": roots,
    }
