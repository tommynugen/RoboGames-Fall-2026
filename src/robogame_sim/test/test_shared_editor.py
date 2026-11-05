"""The shareable arena editor: one HTML file with everything in it."""

import json
import re

from robogame_sim import arena_editor, arena_layout, shared_editor


def test_build_is_one_file():
    page = shared_editor.build()
    # No script or mesh is fetched from a server; only three.js, from
    # unpkg.com through the import map.
    assert not re.search(r'<script src=', page)
    assert 'import(window.arenaView3dUrl' in page
    assert ".server-only { display: none" in page
    data = json.loads(re.search(r"const SHARED = (\{.*?\});\n", page)
                      .group(1).replace("<\\/", "</"))
    assert set(data["layouts"]) == set(arena_layout.list_layouts())
    for kind, part in data["info"]["types"].items():
        assert part["mesh"] in data["meshes"], kind
    # A script's text never closes its own tag.
    for body in re.findall(r"<script>(.*?)</script>", page, re.S):
        assert "</script" not in body


def test_gamemaster_serves_it():
    status, body, kind = arena_editor.get("/api/shared-editor")
    assert status == 200 and kind.startswith("text/html")
    assert body.startswith(b"<!") or b"<html" in body[:200]
