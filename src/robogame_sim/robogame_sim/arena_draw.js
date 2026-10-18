// Draws arena modules as the arena editor does, for the editor and the
// robot console. Each page sets info, the modules from /api/modules, before
// drawing; columns() with no list takes the page's modules.

const NS = "http://www.w3.org/2000/svg";

const cellKey = ([x, y]) => `${x},${y}`;

const isPlatform = type => type === "platform" || type === "platform_2";
const isSide = type => (info.types[type] || {}).mount === "side";
const heightOf = type => (info.types[type] || {}).height || 0;
const inches = height => Math.round(height / 0.0254);

// The way a ramp at a rotation rises, and the side a wall at it faces.
const uphill = rotation => ({ 0: [0, 1], 90: [-1, 0], 180: [0, -1],
                              270: [1, 0] })[rotation];

// The 2 inch cells a module covers, one peg hole in the middle of each and
// two to a square each way, as arena_layout.cells: a platform or ramp
// 2 x 2 of them, and a wall or thwomp the 1 x 2 on the side it faces.
function cellsOf(m) {
  let x = Math.round(2 * m.col), y = Math.round(2 * m.row);
  if (!isSide(m.type)) return [[x, y], [x + 1, y], [x, y + 1], [x + 1, y + 1]];
  const [ux, uy] = uphill(m.rotation);
  if (ux === 0) {
    if (uy > 0) y += 1;
    return [[x, y], [x + 1, y]];
  }
  if (ux > 0) x += 1;
  return [[x, y], [x, y + 1]];
}

// The modules over each cell, bottom first.
function columns(list = modules) {
  const found = new Map();
  for (const m of list) {
    for (const cell of cellsOf(m)) {
      const k = cellKey(cell);
      if (!found.has(k)) found.set(k, []);
      found.get(k).push(m);
    }
  }
  for (const column of found.values()) {
    column.sort((a, b) => a.level - b.level);
  }
  return found;
}

function topAt(cols, cell) {
  const column = cols.get(cellKey(cell));
  return column && column[column.length - 1];
}

// How high a module stands: the platforms under it, on one of its cells.
function standing(m, cols, cell = cellsOf(m)[0]) {
  return (cols.get(cellKey(cell)) || [])
    .filter(p => p.level < m.level)
    .reduce((sum, p) => sum + heightOf(p.type), 0);
}

function el(name, attrs = {}, parent) {
  const node = document.createElementNS(NS, name);
  for (const [k, v] of Object.entries(attrs)) node.setAttribute(k, v);
  if (parent) parent.appendChild(node);
  return node;
}

// The ramp's gradient and the thwomp's stripes, ids starting with prefix.
function defs(svg, prefix) {
  const d = el("defs", {}, svg);
  const g = el("linearGradient", { id: `${prefix}-ramp`, x1: 0, y1: 1,
                                   x2: 0, y2: 0 }, d);
  el("stop", { offset: "0%", "stop-color": "var(--ramp-low)" }, g);
  el("stop", { offset: "100%", "stop-color": "var(--ramp-high)" }, g);
  const p = el("pattern", { id: `${prefix}-stripes`, width: 6, height: 6,
                            patternUnits: "userSpaceOnUse",
                            patternTransform: "rotate(45)" }, d);
  el("rect", { width: 6, height: 6, fill: "var(--thwomp)" }, p);
  el("rect", { width: 2.5, height: 6, fill: "var(--thwomp-edge)",
               opacity: 0.55 }, p);
}

// A module drawn as it is at rotation 0, then turned. The world's rotation
// is anticlockwise seen from above, which is anticlockwise on screen too
// since +y is drawn up, so SVG's clockwise rotate takes the negative.
function turned(parent, x, y, s, rotation, opacity) {
  return el("g", {
    transform: `rotate(${-rotation} ${x + s / 2} ${y + s / 2})`, opacity,
  }, parent);
}

function drawRamp(parent, x, y, s, rotation, prefix, opacity = 1) {
  const g = turned(parent, x, y, s, rotation, opacity);
  el("rect", { x: x + 1, y: y + 1, width: s - 2, height: s - 2, rx: 2,
               fill: `url(#${prefix}-ramp)` }, g);
  el("rect", { x: x + 1, y: y + 1, width: s - 2, height: s * 0.08,
               fill: "var(--platform-edge)", opacity: 0.55 }, g);
  const cx = x + s / 2;
  for (const t of [0.62, 0.38]) {
    el("path", {
      d: `M${cx - s * 0.2} ${y + s * (t + 0.12)} L${cx} ${y + s * t} ` +
         `L${cx + s * 0.2} ${y + s * (t + 0.12)}`,
      fill: "none", stroke: "var(--ramp-arrow)", "stroke-width": s * 0.07,
      "stroke-linecap": "round", "stroke-linejoin": "round",
    }, g);
  }
}

function drawPlatform(parent, x, y, s, opacity = 1) {
  el("rect", { x: x + 1, y: y + 1, width: s - 2, height: s - 2, rx: 2,
               fill: "var(--platform)", stroke: "var(--platform-edge)",
               "stroke-width": s * 0.05, opacity }, parent);
}

function drawPlatform2(parent, x, y, s, opacity = 1) {
  const g = el("g", { opacity }, parent);
  el("rect", { x: x + 1, y: y + 1, width: s - 2, height: s - 2, rx: 2,
               fill: "var(--platform2)", stroke: "var(--platform2-edge)",
               "stroke-width": s * 0.05 }, g);
  el("rect", { x: x + s * 0.18, y: y + s * 0.18, width: s * 0.64,
               height: s * 0.64, rx: 2, fill: "none",
               stroke: "var(--platform2-ink)", "stroke-width": s * 0.03,
               opacity: 0.5 }, g);
  const t = el("text", { x: x + s / 2, y: y + s / 2, "font-size": s * 0.36,
                         "font-weight": 700, fill: "var(--platform2-ink)",
                         "text-anchor": "middle",
                         "dominant-baseline": "central" }, g);
  t.textContent = "2";
}

// A wall or thwomp on the peg line of the side it faces, to scale.
function drawSide(parent, x, y, s, type, rotation, prefix, opacity = 1) {
  const part = info.types[type];
  const scale = s / info.module;
  const depth = Math.max((part.depth || 0.0254) * scale, 3);
  const middle = y + s / 2 - info.peg_line * scale;
  const g = turned(parent, x, y, s, rotation, opacity);
  const thwomp = type === "thwomp";
  el("rect", { x: x + 1.5, y: middle - depth / 2, width: s - 3,
               height: depth, rx: 1.5,
               fill: thwomp ? `url(#${prefix}-stripes)` : "var(--wall)",
               stroke: thwomp ? "var(--thwomp-edge)" : "var(--wall-edge)",
               "stroke-width": thwomp ? 2 : 1.5 }, g);
}

function drawPegs(parent, x, y, s) {
  const scale = s / info.module;
  const off = info.peg_line * scale;
  for (const dx of [-off, off]) {
    for (const dy of [-off, off]) {
      el("circle", { cx: x + s / 2 + dx, cy: y + s / 2 + dy,
                     r: Math.max(s * 0.03, 1), fill: "var(--peg)" }, parent);
    }
  }
}

// How high a stack's top platform is, in the square's corner.
function drawBadge(parent, x, y, s, inches) {
  const w = s * 0.34, h = s * 0.26;
  el("rect", { x: x + s - w - 2, y: y + 2, width: w, height: h, rx: h / 2,
               fill: "var(--panel)", stroke: "var(--ink)",
               "stroke-width": 1, opacity: 0.92 }, parent);
  const t = el("text", { x: x + s - w / 2 - 2, y: y + 2 + h / 2,
                         "font-size": h * 0.72, "font-weight": 700,
                         fill: "var(--ink)", "text-anchor": "middle",
                         "dominant-baseline": "central" }, parent);
  t.textContent = `${inches}"`;
}

function drawModule(parent, type, x, y, s, rotation, prefix, opacity = 1) {
  if (type === "platform") drawPlatform(parent, x, y, s, opacity);
  else if (type === "platform_2") drawPlatform2(parent, x, y, s, opacity);
  else if (type === "ramp") drawRamp(parent, x, y, s, rotation, prefix,
                                     opacity);
  else if (type === "wall" || type === "thwomp") {
    drawSide(parent, x, y, s, type, rotation, prefix, opacity);
  }
}
