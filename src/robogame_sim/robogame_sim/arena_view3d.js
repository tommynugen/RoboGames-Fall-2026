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
// Who to tell when each mesh still loading has come.
const waiting = new Map();

// One geometry for each mesh, shared by every module of that kind; null,
// and done called later, while it loads.
function geometry(mesh, done) {
  if (geometries.get(mesh)) return geometries.get(mesh);
  if (!waiting.has(mesh)) {
    waiting.set(mesh, new Set());
    geometries.set(mesh, null);
    loader.load(`/api/meshes/${mesh}`, g => {
      geometries.set(mesh, g);
      const calls = waiting.get(mesh);
      waiting.delete(mesh);
      for (const call of calls) call();
    });
  }
  waiting.get(mesh).add(done);
  return null;
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
    const height = container.clientHeight || width;
    if (!width) return;
    renderer.setSize(width, height, false);
    camera.aspect = width / height;
    // Taller than wide (a phone): widen the view so the arena's sides stay
    // in frame, as they do in the square view at 40 degrees.
    camera.fov = camera.aspect >= 1 ? 40 : 2 * Math.atan(
      Math.tan(20 * Math.PI / 180) / camera.aspect) * 180 / Math.PI;
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
    // The poses have the tiles' top at z = 0, and base is from the plates.
    const lift = base - (isFloor(m.type) ? 0 : heightOf("floor"));
    mesh.position.set(x + dx, y + dy, z + lift);
    mesh.rotation.z = yaw;
    return mesh;
  }

  // Forget what is built, as a mesh has loaded or the colours changed.
  function rebuild() {
    builtFloor = builtModules = builtGhost = "";
    update();
  }

  // The quadrant plates, boxes as the sim has them, under the tiles.
  function buildFloor() {
    const key = `${size}`;
    if (key === builtFloor || !info.plate) return;
    clear(floor);
    const { colour, thickness, top } = info.plate;
    for (const [x0, x1, y0, y1] of plates(size)) {
      const w = (x1 - x0) * info.module, d = (y1 - y0) * info.module;
      const plate = new THREE.Mesh(new THREE.BoxGeometry(w - 0.002,
        d - 0.002, thickness), material(colour));
      const [cx, cy] = centre((x0 + x1 - 1) / 2, (y0 + y1 - 1) / 2);
      plate.position.set(cx, cy, top - thickness / 2);
      floor.add(plate);
    }
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
    const key = JSON.stringify([hover, tool, deleting, newRotation, stacking,
                                shiftHeld, halves, painting, modules]);
    if (key === builtGhost) return;
    clear(ghost);
    builtGhost = key;
    if (!hover || painting) return;

    const accent = new THREE.Color(css("--accent") || "#2f5fb4");
    const red = new THREE.Color(css("--bad") || "#d23c3c");
    if (deleting) {
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

// -- icons -------------------------------------------------------------------

// The tool and part icons, drawn from each module's mesh: from straight
// above for the top view, and isometric for the 3D view. One small
// renderer draws them all; each picture is kept by type, view and turn.
let iconRenderer = null;
const icons = new Map();
const ICON_PX = 96;

function drawIcon(g, part, view, rotation) {
  if (!iconRenderer) {
    iconRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: true,
                                             preserveDrawingBuffer: true });
    iconRenderer.setSize(ICON_PX, ICON_PX, false);
  }
  const scene = new THREE.Scene();
  scene.add(new THREE.HemisphereLight(0xffffff, 0x666666, 1.4));
  const sun = new THREE.DirectionalLight(0xffffff, 2.4);
  sun.position.set(-0.7, -0.4, 1.2);
  scene.add(sun);
  const mesh = new THREE.Mesh(g, new THREE.MeshStandardMaterial({
    color: new THREE.Color(...(part.colour || [0.6, 0.6, 0.6])),
    roughness: 0.75, metalness: 0.05 }));
  const pose = (part.poses && part.poses[part.turns ? rotation : 0]) ||
               [0, 0, 0, 0];
  mesh.rotation.z = pose[3];
  scene.add(mesh);
  mesh.updateMatrixWorld();
  const box = new THREE.Box3().setFromObject(mesh);
  const mid = box.getCenter(new THREE.Vector3());

  // Straight down for the top view, +y up the picture as on the grid;
  // from the -y, +x side and above for the isometric one.
  const way = view === "top" ? new THREE.Vector3(0, 0, 1)
                             : new THREE.Vector3(1, -1, 1).normalize();
  const camera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.001, 10);
  camera.up.set(0, view === "top" ? 1 : 0, view === "top" ? 0 : 1);
  camera.position.copy(mid).addScaledVector(way, 2);
  camera.lookAt(mid);
  camera.updateMatrixWorld();
  // Fit the box's corners, seen from the camera, with a little margin.
  let reach = 1e-4;
  const inverse = camera.matrixWorldInverse;
  for (let i = 0; i < 8; i++) {
    const p = new THREE.Vector3(i & 1 ? box.max.x : box.min.x,
                                i & 2 ? box.max.y : box.min.y,
                                i & 4 ? box.max.z : box.min.z)
      .applyMatrix4(inverse);
    reach = Math.max(reach, Math.abs(p.x), Math.abs(p.y));
  }
  reach *= 1.08;
  Object.assign(camera, { left: -reach, right: reach, top: reach,
                          bottom: -reach });
  camera.updateProjectionMatrix();
  iconRenderer.render(scene, camera);
  mesh.material.dispose();
  return iconRenderer.domElement.toDataURL("image/png");
}

// A picture of a module, as a data URL, or null while its mesh loads (done
// is called once it has) or for a module with no mesh. view is "top" or
// "iso"; rotation in degrees, for modules that turn.
export function icon(type, view, rotation = 0, done = () => {}) {
  const part = info.types[type];
  if (!part || !part.mesh) return null;
  const key = `${type}|${part.mesh}|${view}|${part.turns ? rotation : 0}`;
  if (icons.has(key)) return icons.get(key);
  const g = geometry(part.mesh, done);
  if (!g) return null;
  icons.set(key, drawIcon(g, part, view, rotation));
  return icons.get(key);
}
