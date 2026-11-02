// The arena editor's 3D view: the layout built from the exported CAD
// meshes, posed as the sim poses them, and edited as the top view is.
// The editor's script holds the layout; this reads its modules, info,
// size, hover, tool and problems, and hands pointer moves and clicks to
// its hoverAt, startStroke and endStroke, as points in squares like col
// and row.

import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";

const loader = new STLLoader();
const geometries = new Map();

// One geometry for each mesh, shared by every module of that kind.
function geometry(mesh, done) {
  if (!geometries.has(mesh)) {
    geometries.set(mesh, null);
    loader.load(`/api/meshes/${mesh}`, g => {
      geometries.set(mesh, g);
      done();
    });
  }
  return geometries.get(mesh);
}

const css = name =>
  getComputedStyle(document.documentElement).getPropertyValue(name).trim();

// Where a module's square is in the world, as arena_layout.centre.
const centre = (col, row) => {
  const half = (size - 1) / 2;
  return [(col - half) * info.module, (row - half) * info.module];
};

export function create(container) {
  const renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setPixelRatio(window.devicePixelRatio);
  container.prepend(renderer.domElement);
  const note = container.querySelector(".note3d");
  note.textContent = "Right-drag to turn, middle-drag to move, wheel to zoom.";

  const scene = new THREE.Scene();
  scene.add(new THREE.HemisphereLight(0xffffff, 0x666666, 1.3));
  const sun = new THREE.DirectionalLight(0xffffff, 2.6);
  sun.position.set(-0.7, -0.4, 1.2);
  scene.add(sun);

  // +z up, as in Gazebo, looking from the -y side as the top view's bottom.
  const camera = new THREE.PerspectiveCamera(40, 1, 0.01, 20);
  camera.up.set(0, 0, 1);
  camera.position.set(0, -1.25, 1.05);
  const controls = new OrbitControls(camera, renderer.domElement);
  // The left button edits; the right one turns the view.
  controls.mouseButtons = { LEFT: -1, MIDDLE: THREE.MOUSE.PAN,
                            RIGHT: THREE.MOUSE.ROTATE };
  controls.touches = { ONE: -1, TWO: THREE.TOUCH.DOLLY_ROTATE };
  controls.maxPolarAngle = Math.PI * 0.49;
  controls.addEventListener("change", draw);

  const floor = new THREE.Group();
  const placed = new THREE.Group();
  const ghost = new THREE.Group();
  scene.add(floor, placed, ghost);
  let builtFloor = "";
  let builtModules = "";
  let builtGhost = "";
  // The mesh of each module on the board, to find the one under a pick.
  let meshOf = new Map();

  function draw() {
    renderer.render(scene, camera);
  }

  function resize() {
    const width = container.clientWidth;
    if (!width) return;
    renderer.setSize(width, width, false);
    camera.aspect = 1;
    camera.updateProjectionMatrix();
    draw();
  }
  new ResizeObserver(resize).observe(container);

  function clear(group) {
    for (const child of [...group.children]) {
      group.remove(child);
      child.material.dispose();
    }
  }

  const material = (rgb, extra = {}) => new THREE.MeshStandardMaterial({
    color: new THREE.Color(...rgb), roughness: 0.75, metalness: 0.05,
    ...extra });

  // A module's mesh posed where it stands, or null until its mesh loads.
  function meshFor(m, base, mat) {
    const part = info.types[m.type];
    const g = part && part.mesh && geometry(part.mesh, rebuild);
    if (!g) return null;
    const [dx, dy, z, yaw] = part.poses[part.turns ? m.rotation : 0];
    const [x, y] = centre(m.col, m.row);
    const mesh = new THREE.Mesh(g, mat);
    mesh.position.set(x + dx, y + dy, z + base);
    mesh.rotation.z = yaw;
    return mesh;
  }

  // Forget what is built, as a mesh has loaded or the colours changed.
  function rebuild() {
    builtFloor = builtModules = builtGhost = "";
    update();
  }

  function buildFloor() {
    const key = `${size}`;
    if (key === builtFloor || !info.floor) return;
    const g = geometry(info.floor.mesh, rebuild);
    if (!g) return;
    clear(floor);
    const [dx, dy, z] = info.floor.pose;
    const mat = material(info.floor.colour);
    for (let row = 0; row < size; row++) {
      for (let col = 0; col < size; col++) {
        const [x, y] = centre(col, row);
        const tile = new THREE.Mesh(g, mat.clone());
        tile.position.set(x + dx, y + dy, z);
        floor.add(tile);
      }
    }
    mat.dispose();
    builtFloor = key;
    const reach = size * info.module;
    controls.target.set(0, 0, 0);
    camera.position.set(0.45 * reach, -1.3 * reach, 0.85 * reach);
    controls.update();
  }

  // Every module, those with problems in red, and stronger red for the
  // one picked in the list of checks.
  function buildModules() {
    const bad = new Set(problems.map(moduleKey));
    const key = JSON.stringify([modules, [...bad], highlighted]);
    if (key === builtModules) return;
    clear(placed);
    meshOf = new Map();
    const cols = columns();
    let missing = false;
    for (const m of modules) {
      const part = info.types[m.type];
      if (!part || !part.colour) continue;
      const k = moduleKey(m);
      const extra = bad.has(k) ? {
        emissive: new THREE.Color(css("--bad") || "#d23c3c"),
        emissiveIntensity: highlighted === k ? 0.9 : 0.45 } : {};
      const mesh = meshFor(m, standing(m, cols), material(part.colour,
                                                          extra));
      if (!mesh) { missing = true; continue; }
      mesh.userData.module = m;
      meshOf.set(m, mesh);
      placed.add(mesh);
    }
    builtModules = missing ? "" : key;
  }

  // What a click would do, as the top view shows it: the tool's module,
  // see-through, red where it does not fit, or the module it would take
  // off, tinted red.
  function buildGhost() {
    const key = JSON.stringify([hover, tool, newRotation, stacking,
                                shiftHeld, halves, painting, modules]);
    if (key === builtGhost) return;
    clear(ghost);
    builtGhost = key;
    if (!hover || painting) return;

    const accent = new THREE.Color(css("--accent") || "#2f5fb4");
    const red = new THREE.Color(css("--bad") || "#d23c3c");
    if (tool === "erase") {
      const m = under(hover);
      const mesh = m && meshOf.get(m);
      if (!mesh) return;
      const copy = mesh.clone();
      copy.material = new THREE.MeshBasicMaterial({
        color: red, transparent: true, opacity: 0.55, depthTest: false });
      ghost.add(copy);
      return;
    }
    const result = plan(hover, false, false);
    const m = result ? result.placed : candidate(hover);
    const base = standing(m, columns(result ? result.modules : modules));
    const mesh = meshFor(m, base + 0.0005, new THREE.MeshStandardMaterial({
      color: result ? accent : red, transparent: true, opacity: 0.5,
      depthWrite: false }));
    if (mesh) ghost.add(mesh); else builtGhost = "";
  }

  function update() {
    buildFloor();
    buildModules();
    buildGhost();
    scene.background = new THREE.Color(css("--bg") || "#f4f5f7");
    draw();
  }

  // -- picking -------------------------------------------------------------

  const ray = new THREE.Raycaster();
  const pointer = new THREE.Vector2();

  // The point in squares under the pointer, on a tile or a module, a hair
  // inside what it hits so the side of a module counts as that module.
  function pointFrom(event) {
    const box = renderer.domElement.getBoundingClientRect();
    pointer.set((event.clientX - box.left) / box.width * 2 - 1,
                -(event.clientY - box.top) / box.height * 2 + 1);
    ray.setFromCamera(pointer, camera);
    const hit = ray.intersectObjects([...placed.children,
                                      ...floor.children])[0];
    if (!hit) return null;
    const normal = hit.face.normal.clone()
      .transformDirection(hit.object.matrixWorld);
    const p = hit.point.addScaledVector(normal, -0.002);
    const half = (size - 1) / 2;
    const x = p.x / info.module + half, y = p.y / info.module + half;
    const inside = v => v >= -0.5 && v < size - 0.5;
    return inside(x) && inside(y) ? { x, y } : null;
  }

  const canvas = renderer.domElement;
  // Where a right-button press started, to tell a click from a turn.
  let rightDown = null;

  canvas.addEventListener("pointerdown", event => {
    shiftHeld = event.shiftKey;
    if (event.button === 2) {
      rightDown = [event.clientX, event.clientY];
      return;
    }
    if (event.button !== 0) return;
    const point = pointFrom(event);
    if (!point) return;
    event.preventDefault();
    canvas.setPointerCapture(event.pointerId);
    startStroke(point, false);
  });
  canvas.addEventListener("pointermove", event => {
    if (event.buttons & 6) return;
    hoverAt(pointFrom(event));
  });
  canvas.addEventListener("pointerup", event => {
    if (event.button === 2 && rightDown) {
      const moved = Math.hypot(event.clientX - rightDown[0],
                               event.clientY - rightDown[1]);
      rightDown = null;
      const point = moved < 5 && pointFrom(event);
      if (point) {
        startStroke(point, true);
        endStroke();
      }
      return;
    }
    if (painting) endStroke();
  });
  canvas.addEventListener("pointercancel", () => {
    if (painting) endStroke();
  });
  canvas.addEventListener("pointerleave", leaveView);
  canvas.addEventListener("contextmenu", event => event.preventDefault());

  window.matchMedia("(prefers-color-scheme: dark)")
    .addEventListener("change", rebuild);

  resize();
  return { update, resize };
}
