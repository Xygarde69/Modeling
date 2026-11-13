#!/usr/bin/env python3
"""URDF -> MJCF interactive GUI. Run: python server.py [urdf_folder_or_file] [--port 8765]"""
import argparse
import glob
import json
import os
import sys
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mjcf_gen  # noqa: E402
from urdf_parser import parse_urdf  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
STATE = {"model": None}


def find_urdf(p):
    p = os.path.abspath(os.path.expanduser(p))
    if os.path.isfile(p):
        return p
    hits = glob.glob(os.path.join(p, "**", "*.urdf"), recursive=True)
    if not hits:
        raise FileNotFoundError(f"no .urdf found under {p}")
    return sorted(hits, key=len)[0]


def model_payload(model):
    links = {}
    for n, l in model["links"].items():
        links[n] = {
            "name": n, "inertial": l["inertial"],
            "visuals": [{k: v for k, v in g.items() if k != "file"} | {"file": g.get("file"),
                        "hasMesh": bool(g.get("path"))} for g in l["visuals"]],
            "collisions": [{"type": g["type"]} for g in l["collisions"]],
        }
    return {"name": model["name"], "path": model["path"], "links": links, "joints": model["joints"],
            "children": model["children"], "roots": model["roots"],
            "config": mjcf_gen.default_config(model), "schema": mjcf_gen.schema_payload()}


class H(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, code, body, ctype="application/json"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode()
        elif isinstance(body, str):
            body = body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        q = {k: v[0] for k, v in parse_qs(u.query).items()}
        try:
            if u.path == "/":
                return self._send(200, open(os.path.join(HERE, "index.html"), "rb").read(), "text/html")
            if u.path == "/api/mesh":
                m = STATE["model"]
                p = os.path.abspath(q["path"])
                allowed = {g["path"] for l in m["links"].values() for g in l["visuals"] + l["collisions"] if g.get("path")}
                if p not in allowed:
                    return self._send(403, {"error": "not a model mesh"})
                return self._send(200, open(p, "rb").read(), "application/octet-stream")
            if u.path == "/api/ls":
                d = os.path.abspath(os.path.expanduser(q.get("path") or "~"))
                if os.path.isfile(d):
                    d = os.path.dirname(d)
                items = []
                for n in sorted(os.listdir(d), key=str.lower):
                    if n.startswith("."):
                        continue
                    f = os.path.join(d, n)
                    if os.path.isdir(f):
                        items.append({"name": n, "dir": True})
                    elif n.lower().endswith(".urdf"):
                        items.append({"name": n, "dir": False})
                return self._send(200, {"path": d, "parent": os.path.dirname(d), "items": items})
            if u.path == "/api/current":
                m = STATE["model"]
                return self._send(200, model_payload(m) if m else {"none": True})
            self._send(404, {"error": "not found"})
        except Exception as e:  # noqa
            self._send(500, {"error": f"{type(e).__name__}: {e}"})

    def do_POST(self):
        u = urlparse(self.path)
        try:
            body = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0))) or b"{}")
            if u.path == "/api/load":
                STATE["model"] = parse_urdf(find_urdf(body["path"]))
                return self._send(200, model_payload(STATE["model"]))
            m = STATE["model"]
            if m is None:
                return self._send(400, {"error": "no URDF loaded"})
            if u.path == "/api/generate":
                xml, gen = mjcf_gen.generate_mjcf(m, body["config"], "absolute")
                return self._send(200, {"xml": xml, "warnings": gen.warnings,
                                        "validation": mjcf_gen.validate(xml)})
            if u.path == "/api/save":
                out = os.path.abspath(os.path.expanduser(body["outDir"]))
                p, warns = mjcf_gen.save_package(m, body["config"], out, body.get("filename") or "model.xml")
                json.dump(body["config"], open(os.path.join(out, "urdf2mjcf_config.json"), "w"), indent=2)
                return self._send(200, {"saved": p, "warnings": warns})
            self._send(404, {"error": "not found"})
        except Exception as e:  # noqa
            import traceback
            traceback.print_exc()
            self._send(500, {"error": f"{type(e).__name__}: {e}"})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("path", nargs="?")
    ap.add_argument("--port", type=int, default=8765)
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    if a.path:
        STATE["model"] = parse_urdf(find_urdf(a.path))
        print("Loaded", STATE["model"]["path"])
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), H)
    url = f"http://127.0.0.1:{a.port}/"
    print("GUI at", url)
    if not a.no_browser:
        threading.Timer(0.5, lambda: webbrowser.open(url)).start()
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
