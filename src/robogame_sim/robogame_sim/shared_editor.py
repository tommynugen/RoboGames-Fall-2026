"""The arena editor as one HTML file, to send to teammates.

    python3 -m robogame_sim.shared_editor [OUT.html]

or Share editor in the Gamemaster's Arena editor tab, which downloads the
same file. It opens in any browser with nothing installed: no ROS, no
Gamemaster, no robots.
Teammates draft layouts in it, kept in their browser when they Save, add
STL parts by dropping them on the page as in the Gamemaster, and Export a
layout as a .json file to send back, with the STL parts it uses inside.
Import in the Gamemaster's editor, or dropping the file on it, opens it
there and adds those parts.

It is arena_editor.html with its scripts, the module sizes, the CAD meshes
for the 3D view and the saved layouts, as starting points, put inside the
page. A small script answers the page's /api/ requests in the browser, so
the page itself is the same as the Gamemaster's, and a file built again
after the editor changes has the changes. It turns a dropped STL into a
part as add_part in arena_layout.py does. The ramp and platform checks,
the sim and renaming or deleting parts are left out, since they need the
Gamemaster: a layout is checked when it is opened there. three.js still
comes from unpkg.com, so the 3D view needs the internet; the top view
does not.
"""

import base64
import json
import os
import sys

from robogame_sim import arena_layout

HERE = os.path.dirname(os.path.realpath(__file__))
FILE_NAME = "robogame-arena-editor.html"

# Answers the page's /api/ requests from what is in the page, and keeps
# saved layouts in the browser's localStorage.
SHIM = r"""
(() => {
  "use strict";
  const SHARED = SHARED_DATA;
  const KEY = "robogame-arena-layouts";
  const own = () => {
    try { return JSON.parse(localStorage.getItem(KEY)) || {}; }
    catch { return {}; }
  };
  const fromBase64 = text => Uint8Array.from(atob(text || ""),
                                             c => c.charCodeAt(0));
  const toBase64 = bytes => {
    let text = "";
    for (let i = 0; i < bytes.length; i += 0x8000) {
      text += String.fromCharCode(...bytes.subarray(i, i + 0x8000));
    }
    return btoa(text);
  };

  // STL parts added here, as add_part in arena_layout.py makes them:
  // {type: {label, stl (base64, metres, z up, centred), saved}}.
  const PARTS = "robogame-arena-parts";
  const ownParts = () => {
    try { return JSON.parse(localStorage.getItem(PARTS)) || {}; }
    catch { return {}; }
  };
  const CELL = SHARED.info.module / 2;
  const ARENA = SHARED.info.size * SHARED.info.module;
  const UNITS = { m: 1, cm: 0.01, mm: 0.001, inch: 0.0254 };
  const TITLE = "robogame_sim part ";

  // Triangles of a binary or ASCII STL, 9 numbers each.
  function parseStl(bytes) {
    const view = new DataView(bytes.buffer, bytes.byteOffset,
                              bytes.byteLength);
    if (bytes.length >= 84) {
      const count = view.getUint32(80, true);
      if (bytes.length === 84 + 50 * count) {
        const out = new Float64Array(count * 9);
        for (let i = 0; i < count; i++) {
          for (let k = 0; k < 9; k++) {
            out[i * 9 + k] = view.getFloat32(84 + i * 50 + 12 + k * 4, true);
          }
        }
        return out;
      }
    }
    const text = new TextDecoder("latin1").decode(bytes);
    const numbers = [...text.matchAll(/vertex\s+(\S+)\s+(\S+)\s+(\S+)/g)]
      .flatMap(m => m.slice(1).map(Number));
    return Float64Array.from(numbers.slice(0, numbers.length -
                                           numbers.length % 9));
  }

  function bounds(points) {
    const lo = [Infinity, Infinity, Infinity];
    const hi = [-Infinity, -Infinity, -Infinity];
    for (let i = 0; i < points.length; i++) {
      lo[i % 3] = Math.min(lo[i % 3], points[i]);
      hi[i % 3] = Math.max(hi[i % 3], points[i]);
    }
    return [lo, hi];
  }

  // A binary STL of the triangles, with the part's name in its header.
  function writeStl(points, label) {
    const count = points.length / 9;
    const bytes = new Uint8Array(84 + 50 * count);
    const view = new DataView(bytes.buffer);
    bytes.set(new TextEncoder().encode((TITLE + label).slice(0, 80)));
    view.setUint32(80, count, true);
    for (let i = 0; i < count; i++) {
      const p = points.subarray(i * 9, i * 9 + 9);
      const u = [p[3] - p[0], p[4] - p[1], p[5] - p[2]];
      const v = [p[6] - p[0], p[7] - p[1], p[8] - p[2]];
      const n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2],
                 u[0] * v[1] - u[1] * v[0]];
      const length = Math.hypot(...n) || 1;
      const at = 84 + i * 50;
      n.forEach((x, k) => view.setFloat32(at + k * 4, x / length, true));
      p.forEach((x, k) => view.setFloat32(at + 12 + k * 4, x, true));
    }
    return bytes;
  }

  // The part's entry in /api/modules, as modules_info() gives it.
  function partInfo(type, part) {
    const [lo, hi] = bounds(parseStl(fromBase64(part.stl)));
    const [w, d, h] = [0, 1, 2].map(k => hi[k] - lo[k]);
    const cells = rotation => {
      const along = [w, d].map(v => Math.max(1, Math.ceil(v / CELL - 1e-3)));
      return rotation % 180 ? [along[1], along[0]] : along;
    };
    const poses = {};
    for (const rotation of [0, 90, 180, 270]) {
      const [cw, cd] = cells(rotation);
      poses[rotation] = [CELL / 2 * (cw % 2), CELL / 2 * (cd % 2), -lo[2],
                         rotation * Math.PI / 180]
        .map(v => Math.round(v * 1e6) / 1e6);
    }
    return { label: part.label, turns: true, builtin: false,
             deleted: false, renamed: false, mount: "block",
             width: w, depth: d, height: h,
             mesh: `parts/${type}.stl?v=${part.saved}`,
             colour: [0.910, 0.627, 0.478], poses, cells: cells(0) };
  }

  function modules() {
    const types = { ...SHARED.info.types };
    for (const [type, part] of Object.entries(ownParts())) {
      try { types[type] = partInfo(type, part); } catch { /* skip it */ }
    }
    return { ...SHARED.info, types };
  }

  // A dropped STL, as add_part: turned so z is up, scaled to metres,
  // centred on x, y = 0 and standing on z = 0.
  function addPart(name, body, units, up) {
    const label = String(name).split(/\s+/).filter(Boolean).join(" ");
    if (!/^[A-Za-z0-9_ -]{1,40}$/.test(label)) {
      throw new Error("names are letters, digits, spaces, - and _ only");
    }
    const type = label.replace(/ /g, "-");
    if ((SHARED.info.types[type] || {}).builtin) {
      throw new Error(`${type} is one of the CAD's modules, pick another ` +
                      `name`);
    }
    const points = parseStl(body);
    if (!points.length || !points.every(Number.isFinite)) {
      throw new Error("that is not an STL with any triangles in it");
    }
    if (up === "y") {
      for (let i = 0; i < points.length; i += 3) {
        const y = points[i + 1];
        points[i + 1] = -points[i + 2];
        points[i + 2] = y;
      }
    }
    let [lo, hi] = bounds(points);
    if (units === "auto") {
      const extent = Math.max(...[0, 1, 2].map(k => hi[k] - lo[k]));
      units = extent > 30 ? "mm" : extent > 1.3 ? "inch" : "m";
    }
    if (!(units in UNITS)) throw new Error(`units must be auto or one of ` +
                                           Object.keys(UNITS).join(", "));
    const scale = UNITS[units];
    const shift = [(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, lo[2]];
    for (let i = 0; i < points.length; i++) {
      points[i] = (points[i] - shift[i % 3]) * scale;
    }
    const size = [0, 1, 2].map(k => (hi[k] - lo[k]) * scale);
    const across = Math.max(...size);
    if (Math.max(size[0], size[1]) > ARENA || size[2] > ARENA) {
      throw new Error(`in ${units} it is ${across.toFixed(2)} m across, ` +
                      `bigger than the arena: pick other units`);
    }
    if (across < 0.005) {
      throw new Error(`in ${units} it is under 5 mm across: pick other ` +
                      `units`);
    }
    const parts = ownParts();
    parts[type] = { label, stl: toBase64(writeStl(points, label)),
                    saved: Date.now() };
    try { localStorage.setItem(PARTS, JSON.stringify(parts)); }
    catch {
      throw new Error("it is too big for this browser to keep");
    }
    return { type, label, units, size };
  }

  const urls = {};
  window.arenaMeshUrl = mesh => {
    if (!urls[mesh]) {
      const own = mesh.match(/^parts\/(.+)\.stl/);
      const text = SHARED.meshes[mesh] ||
        (own && (ownParts()[own[1]] || {}).stl);
      urls[mesh] = URL.createObjectURL(new Blob([fromBase64(text)]));
    }
    return urls[mesh];
  };
  window.arenaView3dUrl = URL.createObjectURL(
    new Blob([SHARED.view3d], { type: "text/javascript" }));
  const reply = (status, body) => new Response(JSON.stringify(body), {
    status, headers: { "Content-Type": "application/json" } });
  const realFetch = window.fetch.bind(window);
  window.fetch = async (path, options = {}) => {
    if (typeof path !== "string" || !path.startsWith("/api/")) {
      return realFetch(path, options);
    }
    const method = (options.method || "GET").toUpperCase();
    const route = path.split("?")[0];
    if (route === "/api/modules") return reply(200, modules());
    if (route.startsWith("/api/parts/") && method === "PUT") {
      const query = new URLSearchParams(path.split("?")[1] || "");
      const body = options.body instanceof ArrayBuffer
        ? new Uint8Array(options.body)
        : new Uint8Array(options.body.buffer, options.body.byteOffset,
                         options.body.byteLength);
      try {
        const part = addPart(
          decodeURIComponent(route.slice("/api/parts/".length)), body,
          query.get("units") || "auto", query.get("up") || "z");
        return reply(200, { part, modules: modules() });
      } catch (error) {
        return reply(400, { error: error.message });
      }
    }
    if (route === "/api/archive") return reply(200, { archive: [] });
    if (route === "/api/check") return reply(200, { problems: [] });
    if (route === "/api/layouts") {
      const names = new Set([...Object.keys(SHARED.layouts),
                             ...Object.keys(own())]);
      return reply(200, {
        layouts: [...names].sort(), folder: "this browser",
        note: "Save keeps layouts in this browser. Export downloads one " +
              "as a file, to send back and open with Import in the " +
              "RoboGame Gamemaster." });
    }
    if (route.startsWith("/api/layouts/")) {
      const name = decodeURIComponent(route.slice("/api/layouts/".length));
      if (method === "PUT") {
        const saved = own();
        saved[name] = JSON.parse(options.body);
        try { localStorage.setItem(KEY, JSON.stringify(saved)); }
        catch {
          return reply(500, { error: "this browser will not keep it, " +
                                     "use Export" });
        }
        return reply(200, { name, path: name, problems: [] });
      }
      const layout = own()[name] || SHARED.layouts[name];
      return layout ? reply(200, layout)
        : reply(404, { error: `no layout ${name}` });
    }
    return reply(404, {
      error: "that needs the RoboGame Gamemaster, not this shared editor" });
  };
})();
"""

STYLE = """
<style>
  .server-only { display: none !important; }
  .shared-note { margin: 0; padding: 8px 20px; font-size: 13px;
                 background: var(--panel); color: var(--muted);
                 border-bottom: 1px solid var(--line); }
</style>
"""

NOTE = """<p class="shared-note">The RoboGame arena editor, on its own.
Save keeps a layout in this browser, and STL files dropped on the page are
added as parts; <b>Export</b> downloads the layout, with its parts, as a
file to send back. The 3D view needs the internet. Ramp and platform checks run
when the layout is opened in the Gamemaster.</p>
"""


def read(name):
    with open(os.path.join(HERE, name), encoding="utf-8") as file:
        return file.read()


def inline_js(source):
    # So no string in a script ends the <script> tag it is in.
    return source.replace("</", "<\\/")


def swap(page, old, new):
    """Replaces the one place old is in the page, so a change to the
    editor that moves it fails the build, not the page."""
    if page.count(old) != 1:
        raise ValueError(f"arena_editor.html has {page.count(old)} of "
                         f"{old!r}, not one")
    return page.replace(old, new)


def mesh_files(info):
    """The 3D view's meshes, by the name the page asks for, as base64."""
    meshes = {}
    for kind, part in arena_layout.PARTS.items():
        if kind not in info["types"]:
            continue
        path = part.mesh if os.path.isabs(part.mesh) else os.path.join(
            arena_layout.mesh_dir(), part.mesh)
        with open(path, "rb") as mesh:
            meshes[info["types"][kind]["mesh"]] = (
                base64.b64encode(mesh.read()).decode())
    return meshes


def saved_layouts():
    layouts = {}
    for name in arena_layout.list_layouts():
        try:
            layouts[name] = arena_layout.load(name)
        except (OSError, ValueError, KeyError):
            continue
    return layouts


def build():
    """The shareable editor's HTML."""
    info = arena_layout.modules_info()
    data = {"info": info, "layouts": saved_layouts(),
            "meshes": mesh_files(info), "view3d": read("arena_view3d.js")}
    shim = SHIM.replace("SHARED_DATA", inline_js(
        json.dumps(data, separators=(",", ":"))), 1)

    page = read("arena_editor.html")
    page = swap(page, "<title>", "<title>RoboGame ")
    page = swap(page, "</head>", STYLE + "</head>")
    page = swap(page, "<body>\n", "<body>\n" + NOTE)
    page = swap(page, '<script src="arena_draw.js"></script>',
                f"<script>{shim}</script>\n"
                f"<script>{inline_js(read('arena_draw.js'))}</script>")
    page = swap(page, '<script src="dialogs.js"></script>',
                f"<script>{inline_js(read('dialogs.js'))}</script>")
    page = swap(page, 'import("./arena_view3d.js")',
                'import(window.arenaView3dUrl || "./arena_view3d.js")')
    return page


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    out = argv[0] if argv else FILE_NAME
    with open(out, "w", encoding="utf-8") as file:
        file.write(build())
    print(f"Wrote {out}: send it to teammates, who open it in a browser "
          f"and Export layouts back.")


if __name__ == "__main__":
    main()
