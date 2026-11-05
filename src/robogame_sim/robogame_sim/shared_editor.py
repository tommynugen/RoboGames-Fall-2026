"""The arena editor as one HTML file, to send to teammates.

    python3 -m robogame_sim.shared_editor [OUT.html]

or Share editor in the Gamemaster's Arena editor tab, which downloads the
same file. It opens in any browser with nothing installed: no ROS, no
Gamemaster, no robots. Teammates draft layouts in it, kept in their
browser when they Save, and Export one as a .json file to send back. Import
in the Gamemaster's editor, or dropping the file on it, opens it there.

It is arena_editor.html with its scripts, the module sizes, the CAD meshes
for the 3D view and the saved layouts, as starting points, put inside the
page. A small script answers the page's /api/ requests in the browser, so
the page itself is the same as the Gamemaster's, and a file built again
after the editor changes has the changes. The ramp and platform checks
and the sim are left out, since they need the Gamemaster: a layout is
checked when it is opened there. three.js still comes from unpkg.com, so
the 3D view needs the internet; the top view does not.
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
  const SHARED = %(data)s;
  const KEY = "robogame-arena-layouts";
  const own = () => {
    try { return JSON.parse(localStorage.getItem(KEY)) || {}; }
    catch { return {}; }
  };
  const urls = {};
  window.arenaMeshUrl = mesh => {
    if (!urls[mesh]) {
      const text = atob(SHARED.meshes[mesh] || "");
      const bytes = Uint8Array.from(text, c => c.charCodeAt(0));
      urls[mesh] = URL.createObjectURL(new Blob([bytes]));
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
    if (route === "/api/modules") return reply(200, SHARED.info);
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
Save keeps a layout in this browser; <b>Export</b> downloads it as a file
to send back. The 3D view needs the internet. Ramp and platform checks run
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
    shim = SHIM % {"data": inline_js(json.dumps(data, separators=(",", ":")))}

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
