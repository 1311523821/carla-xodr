"use strict";
// carla-xodr 描线编辑器前端。
// 只产生 x,y 点；高程、净空、拟合、校验全部由服务端算（避免两套几何实现）。

const cv = document.getElementById("cv");
const ctx = cv.getContext("2d");
const wrap = document.getElementById("canvasWrap");
let META = null, base = new Image(), roads = [], activeId = null;
let sel = null;                 // {ri, pi}
let view = { s: 1, ox: 0, oy: 0 };
let reports = {}, drag = null, undoStack = [];
let dirty = false, lastCheck = 0, savedJson = "[]";

const COLORS = ["#4da3ff", "#ff6ec7", "#7ee787", "#ffd166", "#c792ea",
                "#5dd9d9", "#f97583", "#b3de6f", "#ff9f2e", "#a5d6ff"];

// ------------------------------------------------------------- 坐标换算 --
const w2s = (x, y) => [
  ((x - META.xmin) * META.ppm) * view.s + view.ox,
  ((META.h - 1) - (y - META.ymin) * META.ppm) * view.s + view.oy];
const s2w = (sx, sy) => {
  const ix = (sx - view.ox) / view.s, iy = (sy - view.oy) / view.s;
  return [META.xmin + ix / META.ppm,
          META.ymin + (META.h - 1 - iy) / META.ppm];
};
const colorOf = id => COLORS[((id - 1) % COLORS.length + COLORS.length) % COLORS.length];

// ---------------------------------------------------------------- 渲染 --
function resize() {
  const r = wrap.getBoundingClientRect();
  if (r.width < 2 || r.height < 2) return false;      // 布局还没稳定，稍后再试
  cv.width = Math.round(r.width); cv.height = Math.round(r.height);
  zs.height = Math.max(120, Math.min(cv.height - 60, 520));
  draw(); drawZStrip();
  if (U3D.on) s3dSync();        // GL 画布现在等于视口，窗口一变就得跟着改
  return true;
}

function fitView() {
  if (!resize() || !META) return;
  const s = Math.min(cv.width / META.w, cv.height / META.h) * 0.94;
  if (!(s > 0)) return;
  view.s = s;
  view.ox = (cv.width - META.w * s) / 2;
  view.oy = (cv.height - META.h * s) / 2;
  draw();
}

function draw() {
  if (!META) return;
  // 纹理层在 z-index 更低的 #gl 上，这里若铺不透明底色就会把它整个盖掉：
  // 开 3D 时只能 clear 成透明，关掉时才需要自己垫深色背景。
  if (U3D.on) {
    ctx.clearRect(0, 0, cv.width, cv.height);
  } else {
    ctx.fillStyle = "#0c0d10";
    ctx.fillRect(0, 0, cv.width, cv.height);
  }
  ctx.imageSmoothingEnabled = view.s > 1.5;
  // 轨道模式下 3D 相机在转，2D 这套 w2s 投影和它已经不是同一个视角了。
  // 再画道路/顶点/底图就会钉在屏幕原地骗人，所以只留空画布。
  if (U3D.orbit) return;
  // 高度裁剪开启时不画底图：底图画的是"地板在哪"，而裁剪之后的场景里地板
  // 已经被削掉了，两者叠在一起只会误读。纹理 3D 层负责显示裁剪后的样子。
  if (!Z.on && !U3D.on) {
    ctx.drawImage(base, view.ox, view.oy, META.w * view.s, META.h * view.s);
  }

  if (document.getElementById("showGrid").checked) grid();

  for (const r of roads) {
    const c = colorOf(r.id), isAct = r.id === activeId;
    if (document.getElementById("showBand").checked) band(r, c);
    ctx.lineWidth = isAct ? 2.6 : 1.8;
    ctx.strokeStyle = c;
    ctx.globalAlpha = isAct ? 1 : 0.75;
    path(r, c);
    ctx.globalAlpha = 1;
    verts(r, c, isAct);
    label(r, c);
  }
  problems();
}

function sampleRoad(pts, step) {
  // 仅为渲染粒度而等距采样，不引入任何几何语义。
  // 同时记录每个控制点落在采样数组里的下标，顶点/路名标注要用。
  const out = [], ctrl = [];
  for (let i = 0; i < pts.length - 1; i++) {
    const a = pts[i], b = pts[i + 1];
    const L = Math.hypot(b.x - a.x, b.y - a.y), n = Math.max(1, Math.round(L / step));
    ctrl.push(out.length);
    for (let k = 0; k < n; k++)
      out.push({ x: a.x + (b.x - a.x) * k / n, y: a.y + (b.y - a.y) * k / n });
  }
  if (pts.length) { ctrl.push(out.length); out.push({ ...pts[pts.length - 1] }); }
  return { samples: out, ctrl };
}

function visibleAt(id, i) {
  if (!Z.on) return "ok";
  const st = VIS[id];
  if (!st || st[i] === undefined) return "ok";
  return st[i];
}

function segAlpha(id, i) {
  const a = visibleAt(id, i), b = visibleAt(id, i + 1);
  const worst = (a === "ok" || b === "ok") ? "ok" : (a === "occluded" ? a : b);
  if (worst === "ok") return 1.0;
  return document.getElementById("showOcc").checked ? 0.16 : 0.0;
}

function path(r, col) {
  const S = smp(r.id) || r.points;
  if (S.length < 2) return;
  ctx.strokeStyle = col; ctx.lineWidth = r.id === activeId ? 2.6 : 1.8;
  for (let i = 0; i < S.length - 1; i++) {
    const al = segAlpha(r.id, i);
    if (al <= 0) continue;
    ctx.globalAlpha = al;
    const [x1, y1] = w2s(S[i].x, S[i].y), [x2, y2] = w2s(S[i + 1].x, S[i + 1].y);
    ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  }
  ctx.globalAlpha = 1;
}

function band(r, col) {
  const S = smp(r.id) || r.points;
  if (S.length < 2) return;
  const wl = r.width_left, wr = r.width_right;
  for (let i = 0; i < S.length - 1; i++) {
    const al = segAlpha(r.id, i);
    if (al <= 0) continue;
    const a = S[i], b = S[i + 1];
    const dx = b.x - a.x, dy = b.y - a.y, L = Math.hypot(dx, dy) || 1;
    const nx = -dy / L, ny = dx / L;
    const q = [[a.x + nx * wl, a.y + ny * wl], [b.x + nx * wl, b.y + ny * wl],
               [b.x - nx * wr, b.y - ny * wr], [a.x - nx * wr, a.y - ny * wr]]
      .map(([x, y]) => w2s(x, y));
    ctx.globalAlpha = al * 0.16;
    ctx.fillStyle = col;
    ctx.beginPath();
    q.forEach(([x, y], k) => k ? ctx.lineTo(x, y) : ctx.moveTo(x, y));
    ctx.closePath(); ctx.fill();
  }
  ctx.globalAlpha = 1;
}

function ctrlAlpha(r, i) {
  if (!Z.on) return 1;
  const S = SAMP[r.id], st = VIS[r.id];
  if (!S || !st || st[S.ctrl[i]] === undefined) return 1;
  if (st[S.ctrl[i]] === "ok") return 1;
  return document.getElementById("showOcc").checked ? 0.16 : 0;
}

function verts(r, col, isAct) {
  r.points.forEach((p, i) => {
    const al = ctrlAlpha(r, i);
    if (al <= 0) return;
    const [x, y] = w2s(p.x, p.y);
    ctx.globalAlpha = al;
    const on = sel && sel.ri === roads.indexOf(r) && sel.pi === i;
    ctx.beginPath();
    ctx.arc(x, y, on ? 6 : (isAct ? 4 : 3), 0, 6.2832);
    ctx.fillStyle = on ? "#fff" : col;
    ctx.fill();
    if (isAct || on) { ctx.lineWidth = 1.5; ctx.strokeStyle = "#0c0d10"; ctx.stroke(); }
  });
  ctx.globalAlpha = 1;
}

function label(r, col) {
  if (!r.points.length || ctrlAlpha(r, 0) <= 0) return;
  const [x, y] = w2s(r.points[0].x, r.points[0].y);
  ctx.font = "bold 12px ui-monospace, monospace";
  ctx.fillStyle = col;
  ctx.fillText("R" + r.id + " " + (r.name || ""), x + 8, y - 8);
  // 起点这头画一个指向 +s 的箭头：link_next 说的是"本路终点接对方起点"，
  // 两条平行路必须反向描才成环，而光看线头分不出哪端是 s=0（标签在这里，
  // 但那是文字不是方向）。
  if (r.points.length > 1) {
    const [x1, y1] = w2s(r.points[1].x, r.points[1].y);
    ctx.save();
    ctx.translate(x, y); ctx.rotate(Math.atan2(y1 - y, x1 - x));
    ctx.beginPath(); ctx.moveTo(13, 0); ctx.lineTo(6, -4.5); ctx.lineTo(6, 4.5);
    ctx.closePath(); ctx.fillStyle = col; ctx.fill();
    ctx.restore();
  }
}

function grid() {
  const step = view.s > 12 ? 0.5 : view.s > 5 ? 1 : view.s > 2 ? 2 : 5;
  const [x0, y0] = s2w(0, cv.height), [x1, y1] = s2w(cv.width, 0);
  ctx.strokeStyle = "#ffffff14"; ctx.lineWidth = 1;
  ctx.font = "10px ui-monospace, monospace"; ctx.fillStyle = "#ffffff44";
  for (let x = Math.ceil(x0 / step) * step; x < x1; x += step) {
    const [sx] = w2s(x, 0);
    ctx.beginPath(); ctx.moveTo(sx, 0); ctx.lineTo(sx, cv.height); ctx.stroke();
    if (Math.abs(x % (step * 4)) < 1e-9) ctx.fillText(x.toFixed(0), sx + 2, cv.height - 4);
  }
  for (let y = Math.ceil(y0 / step) * step; y < y1; y += step) {
    const [, sy] = w2s(0, y);
    ctx.beginPath(); ctx.moveTo(0, sy); ctx.lineTo(cv.width, sy); ctx.stroke();
    if (Math.abs(y % (step * 4)) < 1e-9) ctx.fillText(y.toFixed(0), 3, sy - 2);
  }
}

function problems() {
  for (const r of roads) {
    const rep = reports[r.id];
    if (!rep || !rep.clearance_worst_at) continue;
    if (!rep.issues || !rep.issues.length) continue;
    // 问题标记属于道路标注：路被遮住时它也不该浮在屋顶上
    if (ctrlAlpha(r, 0) <= 0) continue;
    const [x, y] = w2s(...rep.clearance_worst_at);
    ctx.strokeStyle = "#ff9f2e"; ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(x, y, 9, 0, 6.2832); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(x - 5, y - 5); ctx.lineTo(x + 5, y + 5);
    ctx.moveTo(x + 5, y - 5); ctx.lineTo(x - 5, y + 5); ctx.stroke();
  }
}

// ------------------------------------------------------------- 命中检测 --
// 顶点吸附半径（屏幕像素）。注意它换算到世界距离随缩放变化：
// 1 m = META.ppm * view.s 像素，缩得越小吸附的世界范围越大，
// 所以密点要放大后再点，或按住 Shift 强制加点。
const SNAP_PX = 6;

function hitVertex(sx, sy, rad = SNAP_PX) {
  for (let ri = 0; ri < roads.length; ri++)
    for (let pi = 0; pi < roads[ri].points.length; pi++) {
      const [x, y] = w2s(roads[ri].points[pi].x, roads[ri].points[pi].y);
      if (Math.hypot(x - sx, y - sy) <= rad) return { ri, pi };
    }
  return null;
}

function hitSegment(sx, sy, rad = 8) {
  let best = null, bd = rad;
  for (let ri = 0; ri < roads.length; ri++) {
    const p = roads[ri].points;
    for (let i = 0; i < p.length - 1; i++) {
      const [ax, ay] = w2s(p[i].x, p[i].y), [bx, by] = w2s(p[i + 1].x, p[i + 1].y);
      const dx = bx - ax, dy = by - ay, L2 = dx * dx + dy * dy;
      if (!L2) continue;
      const t = Math.max(0, Math.min(1, ((sx - ax) * dx + (sy - ay) * dy) / L2));
      const d = Math.hypot(ax + t * dx - sx, ay + t * dy - sy);
      if (d < bd) { bd = d; best = { ri, at: i + 1 }; }
    }
  }
  return best;
}

// ---------------------------------------------------------------- 变更 --
function snapshot() {
  undoStack.push(JSON.stringify(roads));
  if (undoStack.length > 60) undoStack.shift();
}
// 「未保存」灯是常驻的，不能只靠"最后一次动作"来点：撤销回到存档那一版就该灭。
function syncDirty() { dirty = JSON.stringify(roads) !== savedJson; setDirty(dirty); }
function undo() {
  if (!undoStack.length) return status("没有可撤销的操作");
  roads = JSON.parse(undoStack.pop());
  sel = null; activeId = roads.length ? roads[roads.length - 1].id : null;
  // resampleAll 不能漏：draw() 读的是 SAMP 里的采样（折线/车道带/标签都按它画），
  // 只有顶点圈是直接读 r.points。少了这一句，撤销之后线还是撤销前那一版，
  // 而顶点已经在旧位置 —— 同一帧里两套东西对不上，/api/occlusion 也会拿旧采样去问。
  syncDirty(); resampleAll(); renderSide(); draw(); scheduleCheck();
}
function markDirty() {
  syncDirty(); resampleAll();
  status(dirty ? "有未保存改动" : "已改回存档里那一版"); scheduleCheck();
}

function evPos(e) {
  const r = cv.getBoundingClientRect();
  return [e.clientX - r.left, e.clientY - r.top];
}

cv.addEventListener("mousedown", e => {
  if (!META) return;
  const [sx, sy] = evPos(e);
  if (e.button === 1 || e.button === 2) {
    drag = { kind: "pan", sx, sy, ox: view.ox, oy: view.oy };
    e.preventDefault(); return;
  }
  if (e.button !== 0) return;
  const hit = e.shiftKey ? null : hitVertex(sx, sy);
  if (hit) {
    sel = hit; activeId = roads[hit.ri].id;
    drag = { kind: "move", sx, sy, ri: hit.ri, pi: hit.pi, moved: false };
    renderSide(); draw(); return;
  }
  if (e.altKey) {
    const hs = hitSegment(sx, sy);
    if (hs) {
      snapshot();
      const [x, y] = s2w(sx, sy);
      roads[hs.ri].points.splice(hs.at, 0, { x, y });
      sel = { ri: hs.ri, pi: hs.at };
      activeId = roads[hs.ri].id;
      markDirty(); renderSide(); draw(); return;
    }
  }
  if (activeId == null) return status("先点右侧列表选一条路，或按「新建路」");
  const r = roads.find(x => x.id === activeId);
  if (!r) return;
  snapshot();
  const [x, y] = s2w(sx, sy);
  r.points.push({ x, y });
  sel = { ri: roads.indexOf(r), pi: r.points.length - 1 };
  markDirty(); renderSide(); draw();
});

window.addEventListener("mousemove", e => {
  if (!META) return;
  const [sx, sy] = evPos(e);
  const [wx, wy] = s2w(sx, sy);
  const pxPerM = META ? META.ppm * view.s : 0;
  document.getElementById("cursor").textContent =
    "x=" + wx.toFixed(2) + "  y=" + wy.toFixed(2) +
    "  |  " + pxPerM.toFixed(1) + " px/m  吸附≈" +
    (SNAP_PX / Math.max(pxPerM, 1e-6)).toFixed(2) + " m";
  if (!drag) return;
  if (drag.kind === "pan") {
    view.ox = drag.ox + (sx - drag.sx); view.oy = drag.oy + (sy - drag.sy);
    draw(); if (U3D.on && !U3D.orbit) s3dSync();
  } else if (drag.kind === "move") {
    if (!drag.moved) { snapshot(); drag.moved = true; }
    const [x, y] = s2w(sx, sy);
    roads[drag.ri].points[drag.pi] = { x, y };
    markDirty(); draw();
  }
});

window.addEventListener("mouseup", () => { drag = null; });
cv.addEventListener("contextmenu", e => e.preventDefault());

cv.addEventListener("wheel", e => {
  if (!META) return;
  e.preventDefault();
  const [sx, sy] = evPos(e);
  const [wx, wy] = s2w(sx, sy);
  view.s = Math.max(0.15, Math.min(80, view.s * (e.deltaY < 0 ? 1.18 : 1 / 1.18)));
  const [nx, ny] = w2s(wx, wy);
  view.ox += sx - nx; view.oy += sy - ny;
  draw(); if (U3D.on && !U3D.orbit) s3dSync();
}, { passive: false });

window.addEventListener("keydown", e => {
  if (["INPUT", "TEXTAREA"].includes(document.activeElement.tagName)) return;
  if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === "z") { e.preventDefault(); undo(); }
  else if (e.key === "Delete" || e.key === "Backspace") {
    if (!sel) return;
    snapshot();
    roads[sel.ri].points.splice(sel.pi, 1);
    sel = null; markDirty(); renderSide(); draw();
  } else if (e.key === "n" || e.key === "N") { newRoad(); }
  else if (e.key === "Escape") { sel = null; draw(); }
});

// ---------------------------------------------------------------- 侧栏 --
function newRoad() {
  snapshot();
  const id = roads.length ? Math.max(...roads.map(r => r.id)) + 1 : 1;
  roads.push({ id, name: "road_" + String(id).padStart(4, "0"), points: [],
               width_left: 1.1, width_right: 1.1, speed_kmh: 5, link_next: null });
  activeId = id; sel = null; markDirty(); renderSide(); draw();
  status("已建 road " + id + "，在图上左键点击开始描线");
}

function delRoad() {
  if (activeId == null) return status("没选中路");
  snapshot();
  roads = roads.filter(r => r.id !== activeId);
  activeId = roads.length ? roads[0].id : null;
  sel = null; markDirty(); renderSide(); draw();
}

// 客户端点只有 x,y（s/z 由服务端反推）。这里算的是纯显示用的平面折线长，
// 不参与任何几何判定，所以不算重复实现。
function planLen(pts) {
  let s = 0;
  for (let i = 1; i < pts.length; i++)
    s += Math.hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  return s;
}

function renderSide() {
  const list = document.getElementById("roadList");
  list.innerHTML = "";
  if (!roads.length) list.innerHTML = '<div class="dim small">还没有路，点「＋ 新建路」</div>';
  roads.forEach(r => {
    const rep = reports[r.id];
    const el = document.createElement("div");
    el.className = "road" + (r.id === activeId ? " active" : "");
    let badge = "";
    if (rep) {
      if (rep.error) badge = '<span class="badge bad">错误</span>';
      else if (rep.issues && rep.issues.length) badge = '<span class="badge warn">需调整</span>';
      else badge = '<span class="badge">通畅</span>';
    }
    el.style.setProperty("--rc", colorOf(r.id));
    el.innerHTML = '<span class="nm">' + r.id + " · " + (r.name || "") + "</span>" +
      '<span class="sp">' + r.points.length + "点 " +
      planLen(r.points).toFixed(1) + "m</span>" + badge;
    el.onclick = () => { activeId = r.id; sel = null; renderSide(); draw(); };
    list.appendChild(el);
  });

  const p = document.getElementById("props");
  const r = roads.find(x => x.id === activeId);
  if (!r) { p.innerHTML = '<span class="dim">未选中</span>'; return; }
  p.innerHTML =
    '<div class="grid2">' +
    fld("id", r.id, "number") + fld("name", r.name, "text") +
    fld("width_left", r.width_left, "number", "0.05", "0.1") +
    fld("width_right", r.width_right, "number", "0.05", "0.1") +
    fld("speed_kmh", r.speed_kmh, "number", "1", "1") +
    fld("link_next", r.link_next == null ? "" : r.link_next, "number", null, null,
        "终点接哪条路的 id，留空=断头") +
    "</div>";
  p.querySelectorAll("input").forEach(inp => inp.onchange = () => {
    snapshot();
    const k = inp.dataset.k;
    let v = inp.value;
    if (k === "id") v = parseInt(v, 10);
    else if (["width_left", "width_right", "speed_kmh"].includes(k)) v = parseFloat(v);
    else if (k === "link_next") v = v === "" ? null : parseInt(v, 10);
    r[k] = v;
    if (k === "id") activeId = v;
    markDirty(); renderSide(); draw();
  });
}
function fld(k, v, type, min, step, ph) {
  return '<div><label>' + k + '</label><input data-k="' + k + '" type="' + type +
    '" value="' + (v ?? "") + '"' + (min ? ' min="' + min + '"' : "") +
    (step ? ' step="' + step + '"' : "") + (ph ? ' placeholder="' + ph + '"' : "") + "></div>";
}

// ---------------------------------------------------------------- 诊断 --
let checkTimer = null;
function scheduleCheck() {
  clearTimeout(checkTimer);
  document.getElementById("diagAge").textContent = "计算中…";
  checkTimer = setTimeout(runCheck, 550);
}
async function runCheck() {
  const sendable = roads.filter(r => r.points.length >= 2);
  if (!sendable.length) {
    reports = {}; document.getElementById("diag").innerHTML =
      '<span class="dim small">每条路至少 2 个点才会诊断</span>';
    document.getElementById("diagAge").textContent = ""; draw(); return;
  }
  try {
    const res = await fetch("/api/check", { method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ roads: sendable }) });
    const j = await res.json();
    if (j.error) return status("诊断失败: " + j.error, true);
    reports = {}; j.reports.forEach(r => reports[r.id] = r);
    lastCheck = Date.now(); renderDiag(); draw();
  } catch (e) { status("诊断请求失败: " + e, true); }
}
function renderDiag() {
  const d = document.getElementById("diag");
  const rows = Object.values(reports).map(r => {
    if (r.error) return '<div class="row"><b>' + r.name + '</b> <span class="no">' + r.error + "</span></div>";
    const ok = !r.issues.length;
    return '<div class="row"><b>' + r.id + " · " + r.name + '</b> ' +
      '<span class="' + (ok ? "ok" : "no") + '">' + (ok ? "通畅" : r.issues.join("；")) + "</span><br>" +
      '<span class="k">地板 </span><span class="v">' + (r.floor_frac * 100).toFixed(1) + "%</span>" +
      ' <span class="k">最差净空 </span><span class="v">' +
      (r.clearance_worst_m == null ? "3m内无障碍" : r.clearance_worst_m + "m") +
      ' <span class="k">拟合 </span><span class="v">' + r.fit_deviation_m + "m</span>" +
      ' <span class="k">Δz </span><span class="v">' + r.dz_rmse_m + "m</span><br>" +
      '<span class="k">几何段 </span><span class="v">' + r.n_geometry + "</span> " +
      '<span class="v dim">' + JSON.stringify(r.kinds) + "</span>" +
      (r.smooth_disp_m > 0.01 ? ' <span class="k">挪线 </span><span class="v">' + r.smooth_disp_m + "m</span>" : "") +
      "</div>";
  });
  d.innerHTML = rows.join("") || '<span class="dim">—</span>';
  document.getElementById("diagAge").textContent =
    new Date(lastCheck).toLocaleTimeString();
}

// --------------------------------------------------------- 高度裁剪拉条 --
// 单阈值：裁掉所有高于 cut 的点。拉条顶端 = 场景 Z 最大值，底端 = 场景 Z 最小值。
// 从上往下拖，屋顶先被裁掉，再是桌面/隔断顶，最后只剩地面。
const zs = document.getElementById("zstrip");
const zctx = zs.getContext("2d");
let Z = { on: false, cut: 0 };
let VIS = {};                       // 路 id -> 逐采样点的 ok/occluded/cut/off
const U3D = { on: false, orbit: false, booted: false };

function s3d() { return window.Scene3D || null; }
function s3dBoot() {
  if (U3D.booted || !s3d() || !META) return;
  U3D.booted = true;
  window.sceneProgress = ev => {
    const el = document.getElementById("glLoad");
    if (el) el.textContent = ev && ev.total ? "载入纹理 " +
      (100 * ev.loaded / ev.total).toFixed(0) + "%" : "";
  };
  window.onSceneReady = () => {
    const el = document.getElementById("glLoad");
    if (el) el.textContent = "";
    s3d().setView(view); s3d().setEnabled(true); s3dSync();
    status("纹理场景已载入（three.js，来自 FBX 内嵌纹理）");
  };
  window.sceneError = e => {
    // 纹理层没起来就必须退回 2D：不然画布被 clear 成透明，用户看到的是空屏
    U3D.booted = false;
    set3D(false);
    document.getElementById("use3D").checked = false;
    status("纹理场景加载失败: " + (e && e.message ? e.message : e) + "，已退回高度底图", true);
  };
  s3d().init(META);
}
function s3dSync() {
  if (!U3D.booted) return;
  s3d().setView(view);
  s3d().setCut(Z.on ? Z.cut : META.z_max);   // 取消勾选 = 完全不裁，不是把剖切面忘在原位
  s3d().setEnabled(U3D.on);
}
let SAMP = {};                      // 路 id -> 等距采样点（渲染用）

const z2y = z => zs.height - (z - META.z_min) / (META.z_max - META.z_min) * zs.height;
const y2z = y => META.z_min + (zs.height - y) / zs.height * (META.z_max - META.z_min);

function drawZStrip() {
  if (!META) return;
  const w = zs.width, h = zs.height;
  zctx.clearRect(0, 0, w, h);
  zctx.fillStyle = "#101218"; zctx.fillRect(0, 0, w, h);
  if (Z.on) {
    const yc = z2y(Z.cut);
    zctx.fillStyle = "rgba(255,91,91,.16)";        // 被裁掉的上方区域
    zctx.fillRect(0, 0, w, yc);
    zctx.strokeStyle = "#ff5b5b"; zctx.lineWidth = 2;
    zctx.beginPath(); zctx.moveTo(0, yc); zctx.lineTo(w, yc); zctx.stroke();
    zctx.fillStyle = "#ff5b5b";
    for (let x = 3; x < w - 4; x += 9) {            // 裁剪线上方的斜纹
      zctx.beginPath(); zctx.moveTo(x, yc - 2); zctx.lineTo(x + 5, yc - 8);
      zctx.lineTo(x + 8, yc - 8); zctx.lineTo(x + 3, yc - 2); zctx.fill();
    }
    zctx.fillStyle = "#fff"; zctx.fillRect(0, yc - 1, w, 2);
    zctx.fillStyle = "#4da3ff";
    zctx.beginPath();                                        // 右侧三角把手
    zctx.moveTo(w - 1, yc - 6); zctx.lineTo(w - 1, yc + 6); zctx.lineTo(w - 9, yc);
    zctx.fill();
  }
  zctx.fillStyle = "#8b93a2"; zctx.font = "9px ui-monospace, monospace";
  zctx.fillText(META.z_max.toFixed(1), 3, 10);               // 100% = Z 最大
  zctx.fillText(META.z_min.toFixed(1), 3, h - 3);            // 0%   = Z 最小
  zctx.strokeStyle = "#3a4150"; zctx.lineWidth = 1;
  for (let i = 1; i < 4; i++) {
    const y = Math.round(h * i / 4) + .5;
    zctx.beginPath(); zctx.moveTo(w - 10, y); zctx.lineTo(w - 2, y); zctx.stroke();
  }
}

function resampleAll() {
  SAMP = {};
  for (const r of roads) SAMP[r.id] = sampleRoad(r.points, 0.5);
}
const smp = id => (SAMP[id] ? SAMP[id].samples : null);

let occTimer = null, occVer = 0;
function refreshOcclusion() {
  // 拖动时每帧都发一次请求会打爆服务端，而且响应回来的顺序不保证：
  // 旧阈值的结果后到就会盖掉新结果。所以去抖 + 版本号，只认最新那次。
  const v = ++occVer;
  clearTimeout(occTimer);
  if (!Z.on) { VIS = {}; draw(); return; }
  occTimer = setTimeout(() => postOcclusion(v), 120);
}

async function postOcclusion(v) {
  if (v !== occVer) return;
  const payload = { cut: Z.cut, roads: roads.filter(r => r.points.length >= 2)
    .map(r => ({ id: r.id, pts: SAMP[r.id].samples.map(p => [p.x, p.y]) })) };
  if (!payload.roads.length) { VIS = {}; draw(); return; }
  try {
    const res = await fetch("/api/occlusion", { method: "POST",
      headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload) });
    const j = await res.json();
    if (v !== occVer) return;                 // 等的时候又拖了，结果已过期
    VIS = {};
    (j.roads || []).forEach(e => { VIS[e.id] = e.status; });
    const n = Object.values(VIS).flat().filter(x => x === "occluded").length;
    if (n) {
      let s = "剖切阈值 " + Z.cut.toFixed(2) + " m：有 " + n + " 段路被更高的面遮住";
      const lo = j.occl_lo, hi = j.occl_hi;
      if (lo != null && hi != null) {
        s += "（遮住它的面在 " + lo.toFixed(2) + " ~ " + hi.toFixed(2) + " m）";
        if (hi - lo < 0.2) s += " —— 阈值正卡在这层自己的起伏里，" +
          "往下拖到 " + (lo - 0.05).toFixed(2) + " 就整层干净了";
      }
      status(s);
    }
    draw();
  } catch (e) { status("遮挡查询失败: " + e, true); }
}

let zDrag = false;
function zSet(z) {
  Z.cut = Math.max(META.z_min, Math.min(META.z_max, z));
  document.getElementById("zCut").value = Z.cut.toFixed(2);
  // 读数就写在标尺帽上，只给数字：54px 的表头撑不下一整句话，
  // "高于此的都裁掉"在提示列表和拉条的 title 里都有。
  document.getElementById("zRead").textContent = Z.cut.toFixed(2) + " m";
  drawZStrip();
  if (U3D.on) s3dSync();
  refreshOcclusion();
}
function zPick(e) {
  const r = zs.getBoundingClientRect();
  zSet(y2z(e.clientY - r.top));
}
// 用 Pointer Events + 指针捕获：mousedown/全局 mouseup 那套在"释放发生在窗口外"时
// 收不到 mouseup，拖拽状态会永久残留，之后鼠标只是滑过竖条就会改阈值。
zs.addEventListener("pointerdown", e => {
  if (!META) return;
  e.preventDefault();
  zs.setPointerCapture(e.pointerId);
  document.getElementById("zOn").checked = Z.on = true;
  zDrag = true; zPick(e);
});
zs.addEventListener("pointermove", e => { if (zDrag) zPick(e); });
zs.addEventListener("pointerup", e => {
  zDrag = false;
  if (zs.hasPointerCapture(e.pointerId)) zs.releasePointerCapture(e.pointerId);
});
zs.addEventListener("pointercancel", () => { zDrag = false; });
zs.addEventListener("wheel", e => {
  if (!META) return;
  e.preventDefault();
  // 按**滚动距离**计价，不按事件个数：触控板/惯性滚动一次手势能发上百个事件，
  // 每个都算 ±0.05 m 的话一滑阈值就掉好几米（实测 116 个事件拽下去 5.8 m）。
  const px = e.deltaMode === 1 ? e.deltaY * 16 : e.deltaMode === 2 ? e.deltaY * 100 : e.deltaY;
  const step = Math.max(-0.2, Math.min(0.2, -px * 0.0005));   // 一格(≈100px) = 0.05 m
  if (step) zSet(Z.cut + step);
}, { passive: false });

function initZ() {
  if (!META) return;
  Z.on = document.getElementById("zOn").checked;
  Z.cut = META.z_max;                    // 顶端 = 什么都不裁
  document.getElementById("zCut").value = Z.cut.toFixed(2);
  document.getElementById("zRead").textContent = Z.cut.toFixed(2) + " m";
  drawZStrip();
}


// ---------------------------------------------------------------- 存档 --
function status(msg, err) {
  // 消息写进 #statusMsg，#status 本身还带着左侧的常驻读数（未保存灯、坐标），
  // 不能整体 textContent 覆盖掉。
  document.getElementById("statusMsg").textContent = msg;
  document.getElementById("status").className = err ? "err" : "";
}
function setDirty(on) { document.getElementById("chipDirty").hidden = !on; }
async function save() {
  // 走 jfetch 而不是裸 fetch + res.json()：服务端 400/500 会返回 HTML，res.json()
  // 直接抛异常、状态栏就停在原地，看起来像"点了没反应"。可复现的例子是把属性框里
  // 的 id 清空 -> parseInt 得 NaN -> JSON 里是 null -> 服务端 int(None) 抛
  // TypeError -> 500 + HTML。兄弟调用 runCheck/align/postOcclusion 一直是这么做的。
  const { j, err } = await jfetch("/api/state", { method: "POST",
    headers: { "Content-Type": "application/json" }, body: JSON.stringify({ roads }) });
  if (err || !j) {
    status("保存失败: " + (err || "服务端没有返回内容"), true);
    return false;
  }
  if (j.error) { status("保存失败: " + j.error, true); return false; }
  savedJson = JSON.stringify(roads); syncDirty();
  j.reports.forEach(r => reports[r.id] = r);
  renderDiag(); renderSide(); draw();
  status("已保存 " + (META.traces || "traces.json") + "（" + j.roads +
    " 条路，z 已从扫描网格反推）");
  return true;
}
async function jfetch(url, opt) {
  // 静态 JS 每次现读磁盘，Python 路由却活在启动那一刻的进程里，两者会不同步：
  // 新按钮配上旧进程就是 404/405 的 HTML，直接 .json() 会抛异常、状态栏永远停在
  // "部署中…"。所以先按文本收，解析失败就把"服务端是旧的"说出来。
  const r = await fetch(url, opt);
  const t = await r.text();
  try { return { j: JSON.parse(t), ok: r.ok }; }
  catch (e) {
    // 只有 404/405 才是"接口不存在、进程旧"。其它状态码下解析失败说明服务端
    // 真的抛了异常（Flask 回的是 HTML 错误页），别把用户指去重启进程。
    if (r.status === 404 || r.status === 405) {
      return { j: null, ok: false, err: "服务端没有这个接口（HTTP " + r.status +
          "）—— 编辑器进程是旧的，重启 trace_editor.py 才有" };
    }
    return { j: null, ok: false, err: "服务端返回了非 JSON（HTTP " + r.status +
        "）—— 多半是服务端内部报错，看跑 trace_editor.py 那个终端的 traceback" };
  }
}
async function emit() {
  // 存盘失败就停：/api/emit 读的是磁盘上的 traces.json，继续跑出来的 xodr 和屏幕
  // 上不是同一套东西，而状态栏看着还是成功的。
  if (dirty && !await save()) return;
  status("生成中…");
  const { j, err } = await jfetch("/api/emit", { method: "POST" });
  if (err || j.error) return status("生成失败: " + (err || j.error), true);
  const ck = j.checks || { total: 0, fails: ["拿不到校验结果"] };
  const bad = (ck.fails || []).length;
  status("已生成 " + j.files.join("  ") +
    (j.staged ? "   已投放 " + j.staged + "/（fbx+xodr 同名配对，可直接 make import）" : "") +
    "   校验 " + ck.total + " 项" + (bad ? "，" + bad + " 项未过：" + ck.fails.join(" ｜ ")
                                       : "全过（含 CARLA 真解析 + 闭环比对）") +
    "   CARLA 里那份: " + (j.deploy ? j.deploy.state : "未知") +
    (j.deploy && j.deploy.state === "未导入"
      ? "　首次要把 FBX 导成关卡：点「⎘ make import」复制命令" : "") +
    // 未标定发射时两版 xodr 数值相同这件事必须说出来：图上看着完全正常，
    // 只有拿到 CARLA 里跑才会发现路和扫描面不是同一帧。
    (j.frame_note ? "　⚠ " + j.frame_note : ""), bad > 0);
  refreshDeploy();
}
async function refreshDeploy() {
  // 只问状态不改文件：让按钮自己说清"Content 里现在装的是不是这份"
  const b = document.getElementById("btnDeploy");
  const { j, err } = await jfetch("/api/deploy");
  if (err) { b.textContent = "部署到 CARLA（服务端过旧）"; b.classList.add("warn"); return; }
  if (!j || !j.target) return;
  const none = j.state === "未导入";
  b.dataset.importCmd = j.import_cmd || "";
  b.disabled = none;
  b.classList.toggle("warn", !none && j.state !== "一致");
  b.classList.toggle("ok", j.state === "一致");
  b.textContent = none ? "部署到 CARLA（还没 make import）"
    : j.state === "一致" ? "重新部署到 CARLA"
    : "部署到 CARLA（Content 里那份" + j.state + "）";
}
async function copyImport() {
  const cmd = document.getElementById("btnDeploy").dataset.importCmd || "";
  if (!cmd) return status("还没拿到命令：服务端没回 import_cmd（编辑器进程偏旧，重启一次）", true);
  let ok = false;
  try { await navigator.clipboard.writeText(cmd); ok = true; } catch (e) { ok = false; }
  if (!ok) {
    // 非安全上下文/无焦点时 Clipboard API 会直接 reject，退回 execCommand 再试一次
    const ta = document.createElement("textarea");
    ta.value = cmd; ta.style.position = "fixed"; ta.style.opacity = "0";
    document.body.append(ta); ta.select();
    try { ok = document.execCommand("copy"); } catch (e) { ok = false; }
    ta.remove();
  }
  // 两条都不成也要能拿到：命令原样摊在状态栏里，选中就能复制
  status((ok ? "已复制：" : "剪贴板不可用，手动选中：") + cmd);
}
async function deploy() {
  status("部署中…");
  const { j, err } = await jfetch("/api/deploy", { method: "POST" });
  if (err || j.error) return status("部署失败: " + (err || j.error), true);
  status("已部署 " + j.target + "（md5 " + j.md5.slice(0, 8) +
    (j.md5_ok ? "" : "，⚠ 与源文件不一致") + "）" +
    (j.removed.length ? "，清掉 " + j.removed.length + " 份陈旧缓存: " + j.removed.join("  ")
                      : "，缓存本来是干净的") +
    "。回 UE 重开一次 Play 才生效");
  refreshDeploy();
}
function kv(k, v, cls) {
  const row = document.createElement("div"); row.className = "row";
  const a = document.createElement("span"); a.className = "k"; a.textContent = k + " ";
  const b = document.createElement("span"); b.className = "v" + (cls ? " " + cls : "");
  b.textContent = v;
  row.append(a, b); return row;
}
async function align() {
  status("对齐检查中…（约 3 秒，要读扫描网格）");
  const { j, err } = await jfetch("/api/align", { method: "POST" });
  if (err || j.error) return status("对齐检查失败: " + (err || j.error), true);
  const rows = [kv("Δz RMSE", j.rmse.toFixed(4) + " m"),
    kv("p50 / p95 / max", j.p50.toFixed(4) + " / " + j.p95.toFixed(4) + " / " +
       j.max.toFixed(4) + " m"),
    kv("内点率(|Δz|<" + j.inlier_m.toFixed(2) + ")", (100 * j.inlier_frac).toFixed(1) + "%"),
    kv("采样点", j.n + " 个，地板可查 " + (100 * j.floor_frac).toFixed(1) + "%")];
  j.roads.forEach(r => rows.push(kv("road " + r.id, r.few ? "可对比点 " + r.n + " —— 太少"
    : "n=" + r.n + "  RMSE " + r.rmse.toFixed(4) + "  中位 " +
      (r.med >= 0 ? "+" : "") + r.med.toFixed(3))));
  rows.push(kv("验收", "最差 " + j.worst.toFixed(4) + " ≤ " + j.limit.toFixed(2) +
    " → " + (j.pass ? "通过" : "未通过"), j.pass ? "ok" : "no"));
  const box = document.getElementById("align");
  box.className = ""; box.replaceChildren(...rows);
  document.getElementById("alignAge").textContent =
    new Date().toLocaleTimeString("zh-CN", { hour12: false });
  status("对齐检查: 最差 Δz RMSE=" + j.worst.toFixed(4) + " m（阈值 " + j.limit.toFixed(2) +
    "）" + (j.pass ? "通过" : "未通过") + "　A_S2W calibrated=" + j.calibrated +
    "，用的 " + j.frame_json);
}
async function toggleCfg() {  const box = document.getElementById("deployCfg");
  box.hidden = !box.hidden;
  if (!box.hidden) await loadCfg();
}
async function loadCfg() {
  const { j, err } = await jfetch("/api/config");
  const msg = document.getElementById("cfgMsg");
  if (err) { msg.textContent = err; msg.className = "err"; return; }
  document.getElementById("cfgRoot").value = j.carla_root;
  document.getElementById("cfgPkg").value = j.package;
  document.getElementById("cfgCache").value = j.client_cache;
  document.getElementById("cfgFile").textContent = j.file;
  document.getElementById("cfgPreview").textContent = "→ " + (j.preview || "载入场景后可预览");
  msg.textContent = ""; msg.className = "dim";
}
async function saveCfg() {
  const body = { carla_root: document.getElementById("cfgRoot").value,
                 package: document.getElementById("cfgPkg").value,
                 client_cache: document.getElementById("cfgCache").value };
  const msg = document.getElementById("cfgMsg");
  const { j, err } = await jfetch("/api/config", { method: "POST",
    headers: { "Content-Type": "application/json" }, body: JSON.stringify(body) });
  if (err || j.error) { msg.textContent = err || j.error; msg.className = "err"; return; }
  document.getElementById("cfgPreview").textContent = "→ " + (j.preview || "");
  msg.textContent = "已写入 " + j.saved; msg.className = "dim";
  refreshDeploy();
}

document.getElementById("btnNew").onclick = newRoad;
document.getElementById("btnDel").onclick = delRoad;
document.getElementById("btnUndo").onclick = undo;
document.getElementById("btnSave").onclick = save;
document.getElementById("btnEmit").onclick = emit;
document.getElementById("btnAlign").onclick = align;
document.getElementById("btnDeploy").onclick = deploy;
document.getElementById("btnImport").onclick = copyImport;
document.getElementById("btnCfg").onclick = toggleCfg;
document.getElementById("cfgSave").onclick = saveCfg;
document.getElementById("showBand").onchange = draw;
document.getElementById("showGrid").onchange = draw;
document.getElementById("showOcc").onchange = draw;
function set3D(on) {
  U3D.on = on;
  if (on && !U3D.booted) { if (!s3d()) return status("three.js 模块未就绪", true); s3dBoot(); }
  if (on && U3D.booted) s3dSync();
  if (!on && U3D.booted) s3d().setOrbit(false);
  if (!on) {
    U3D.orbit = false;
    document.getElementById("orbit").checked = false;
    cv.style.pointerEvents = "auto";
  }
  draw();
}
document.getElementById("use3D").onchange = e => set3D(e.target.checked);
document.getElementById("orbit").onchange = e => {
  U3D.orbit = e.target.checked;
  if (!U3D.on || !U3D.booted) { e.target.checked = false; U3D.orbit = false;
    return status("先勾选「显示纹理场景」才能轨道查看"); }
  // #cv 叠在 #gl 之上，不摘掉它的事件，拖拽就落不到 OrbitControls 上
  cv.style.pointerEvents = U3D.orbit ? "none" : "auto";
  s3d().setOrbit(U3D.orbit);
  document.getElementById("canvasWrap").style.cursor = U3D.orbit ? "grab" : "crosshair";
  document.getElementById("cursor").textContent =
    U3D.orbit ? "轨道模式，2D 叠加已隐藏" : "x=– y=–";
  draw();
  status(U3D.orbit ? "轨道模式：拖拽旋转、滚轮缩放，只用于观察；描线请切回俯视"
                   : "回到俯视，与 2D 画布逐像素对齐");
};
document.getElementById("zOn").onchange = e => {
  Z.on = e.target.checked; if (!Z.on) VIS = {};
  drawZStrip(); if (U3D.on) s3dSync(); draw();
  if (Z.on) refreshOcclusion();
  status(Z.on ? "高度裁剪开：高于阈值 " + Z.cut.toFixed(2) +
                " m 的点全部隐藏，从上往下拖可依次剥掉屋顶、桌面、家具"
              : "高度裁剪关");
};
document.getElementById("zCut").onchange = e => {
  const v = parseFloat(e.target.value);
  if (META && Number.isFinite(v)) {
    if (!Z.on) { Z.on = true; document.getElementById("zOn").checked = true; }
    zSet(v);
  }
};
window.addEventListener("resize", resize);
window.addEventListener("beforeunload", e => { if (dirty) e.preventDefault(); });

// 调试/自动化用：在 devtools 里可直接读 window.__editor
window.__editor = {
  get roads() { return roads; },
  get view() { return view; },
  get meta() { return META; },
  get activeId() { return activeId; },
  set activeId(v) { activeId = v; },
  w2s, s2w, newRoad, save, fitView, draw
};

// ------------------------------------------------------- 加载数据（FBX）--
// 载入是半分钟量级的后台活（Blender 烘 GLB + 缩纹理 + 射线场 + 底图），所以
// 进度必须报"走到哪一步、这步花了多久"，一个转圈会让人以为卡死。
// 路径合法性、越界检查全在服务端，这里只负责显示。
const DLG = { dir: null, parent: null, list: [], sel: null, cur: 0, timer: 0 };

function fmtMB(b) { return (b / 1e6).toFixed(0) + " MB"; }
function dlgEl(id) { return document.getElementById(id); }

function dlgMsg(text, bad) {
  const m = dlgEl("dlgMsg");
  m.textContent = text;
  m.className = bad ? "bad" : "dim";
}

async function dlgGoto(dir) {
  const j = await (await fetch("/api/scanlist" +
    (dir ? "?dir=" + encodeURIComponent(dir) : ""))).json();
  if (j.error) return dlgMsg(j.error, true);
  DLG.dir = j.dir; DLG.parent = j.parent; DLG.sel = null; DLG.cur = 0;
  DLG.list = j.dirs.map(d => ({ kind: "dir", name: d })).concat(
    j.fbx.map(f => ({ kind: "fbx", name: f.name, bytes: f.bytes })));
  const parts = j.dir.split("/");
  const shown = parts.length > 3 ? "…/" + parts.slice(-2).join("/") : j.dir;
  const el = dlgEl("dlgPath");
  el.textContent = shown;
  el.title = j.dir;                              // 完整路径在悬停里
  dlgEl("dlgUp").disabled = !j.parent;
  dlgEl("dlgOk").disabled = true;
  dlgMsg(DLG.list.length ? "选中一个 .fbx，再点「载入所选」"
                         : "这个目录下没有 .fbx，换个目录看看");
  renderDlgList();
  dlgEl("dlgList").focus();
}

function renderDlgList() {
  const ul = dlgEl("dlgList");
  ul.innerHTML = "";
  if (!DLG.list.length) {
    const li = document.createElement("li");
    li.className = "empty"; li.textContent = "空目录";
    ul.appendChild(li); return;
  }
  DLG.list.forEach((e, i) => {
    const li = document.createElement("li");
    li.className = e.kind + (i === DLG.cur ? " cur" : "") +
                   (e.kind === "fbx" && e.name === DLG.sel ? " sel" : "");
    const nm = document.createElement("span");
    nm.className = "nm";
    nm.textContent = e.kind === "dir" ? e.name + "/" : e.name;
    li.appendChild(nm);
    if (e.kind === "fbx") {
      const s = document.createElement("span");
      s.className = "sz"; s.textContent = fmtMB(e.bytes);
      li.appendChild(s);
    }
    li.onclick = () => dlgPick(i);
    ul.appendChild(li);
  });
}

function dlgPick(i) {
  const e = DLG.list[i];
  if (!e) return;
  DLG.cur = i;
  if (e.kind === "dir") return dlgGoto(DLG.dir + "/" + e.name);
  DLG.sel = e.name;
  dlgEl("dlgOk").disabled = false;
  dlgMsg("将载入 " + e.name);
  renderDlgList();
}

function openDlg() {
  dlgEl("mask").hidden = false;
  dlgEl("dlgRun").hidden = true;
  dlgEl("dlgList").hidden = false;
  dlgEl("dlgTitle").textContent = "选择扫描 FBX";
  dlgEl("dlgUp").hidden = dlgEl("dlgOk").hidden = false;
  for (const id of ["dlgUp", "dlgOk", "dlgClose"]) dlgEl(id).disabled = id === "dlgOk";
  dlgGoto(DLG.dir);
}
function closeDlg() {
  clearInterval(DLG.timer); DLG.timer = 0;
  dlgEl("mask").hidden = true;
}

async function dlgLoad() {
  if (!DLG.sel) return;
  const { j, err } = await jfetch("/api/load", { method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ path: DLG.dir + "/" + DLG.sel }) });
  if (err || !j) return dlgMsg("载入失败: " + (err || "服务端没有返回内容"), true);
  if (j.error) return dlgMsg(j.error, true);
  dlgEl("dlgTitle").textContent = "正在载入 " + DLG.sel;
  dlgEl("dlgList").hidden = true;
  dlgEl("dlgRun").hidden = false;
  // 跑起来之后"上一级/载入所选"没有意义，灰着占位不如收掉
  dlgEl("dlgUp").hidden = dlgEl("dlgOk").hidden = true;
  dlgEl("dlgClose").disabled = true;
  dlgMsg("这步要约半分钟（换过的 FBX 要重烘，用缓存则一两秒）");
  clearInterval(DLG.timer);
  DLG.timer = setInterval(pollLoad, 600);
  pollLoad();
}

async function pollLoad() {
  const j = await (await fetch("/api/loadstatus")).json();
  const n = j.phases.length;
  dlgEl("dlgBar").style.width = (100 * Math.min(j.phase, n) / n).toFixed(1) + "%";
  dlgEl("dlgDetail").textContent = j.error || j.detail || "";
  const ol = dlgEl("dlgPhases");
  ol.innerHTML = "";
  j.phases.forEach((name, i) => {
    const li = document.createElement("li");
    const st = j.marks[i] !== undefined ? "ok" : (i === j.phase ? "on" : "");
    li.className = st;
    const mk = document.createElement("span");
    mk.className = "mk";
    mk.textContent = st === "ok" ? "✓" : st === "on" ? "▸" : "·";
    const lb = document.createElement("span");
    lb.textContent = name;
    li.appendChild(mk); li.appendChild(lb);
    if (st === "ok") {
      const t = document.createElement("span");
      t.className = "t";
      t.textContent = (j.marks[i] - (j.marks[i - 1] || 0)).toFixed(1) + "s";
      li.appendChild(t);
    }
    ol.appendChild(li);
  });
  if (j.state === "error") {
    clearInterval(DLG.timer); DLG.timer = 0;
    dlgEl("dlgClose").disabled = false;
    dlgMsg("载入失败，换一个文件或看下方原因", true);
    return;
  }
  if (j.state === "done") {
    clearInterval(DLG.timer); DLG.timer = 0;
    dlgEl("dlgBar").style.width = "100%";
    dlgMsg("已载入，正在打开场景…");
    setTimeout(() => location.reload(), 900);
  }
}

dlgEl("btnLoad").onclick = openDlg;
dlgEl("btnLoad2").onclick = openDlg;
dlgEl("dlgClose").onclick = closeDlg;
dlgEl("dlgOk").onclick = dlgLoad;
dlgEl("dlgUp").onclick = () => DLG.parent && dlgGoto(DLG.parent);
dlgEl("dlgList").addEventListener("keydown", e => {
  if (e.key === "ArrowDown" || e.key === "ArrowUp") {
    e.preventDefault();
    dlgPick(Math.max(0, Math.min(DLG.list.length - 1,
                                DLG.cur + (e.key === "ArrowDown" ? 1 : -1))));
  } else if (e.key === "Enter") { e.preventDefault(); dlgPick(DLG.cur); }
});
dlgEl("mask").addEventListener("keydown", e => {
  if (e.key === "Escape" && !dlgEl("dlgClose").disabled) closeDlg();
});
dlgEl("mask").addEventListener("click", e => {
  if (e.target === dlgEl("mask") && !dlgEl("dlgClose").disabled) closeDlg();
});

// ---------------------------------------------------------------- 启动 --
window.addEventListener("error", e =>
  status("JS 错误: " + e.message + "  @" + (e.filename || "?") + ":" + (e.lineno || "?"), true));

async function boot() {
  try {
    await bootInner();
  } catch (e) {
    status("启动失败: " + (e && e.stack ? e.stack : e), true);
  }
}

async function bootInner() {
  const meta = await (await fetch("/api/meta")).json();
  if (!meta.loaded) {
    document.getElementById("empty").hidden = false;
    document.getElementById("btnLoad").focus();
    status("还没有加载场景：点「加载数据」选一个扫描 FBX");
    return;
  }
  document.body.dataset.loaded = "1";
  META = meta;
  // 必须先挂 onload 再设 src：本地缓存的图可能在处理器装好前就加载完，
  // 那样 onload 永不触发，boot 会卡死在这里（canvas 停在默认 300x150）。
  await new Promise(r => {
    base.onload = r;
    base.onerror = () => { status("底图 basemap.png 加载失败", true); r(); };
    base.src = "assets/basemap.png";
    if (base.complete && base.naturalWidth) r();
  });
  const st = await (await fetch("/api/state")).json();
  roads = (st.roads || []).map(r => ({
    id: r.id, name: r.name, width_left: r.width_left, width_right: r.width_right,
    speed_kmh: r.speed_kmh, link_next: r.link_next ?? null,
    points: (r.points || []).map(p => ({ x: p.x, y: p.y })) }));
  // 存档是上一份场景的就把路先放下：把 A 场景的线画到 B 场景上，出来的 xodr 是废的，
  // 而且看上去完全正常。文件本身不动，重新载入 A 就能看到。
  if (roads.length && st.scene && st.scene !== META.fbx) {
    status("traces.json 里的 " + roads.length + " 条路属于 " +
      st.scene.split("/").pop() + "，与当前场景 " + META.fbx.split("/").pop() +
      " 不是同一份，已先不显示（文件未改动，载入那份场景即可找回）", true);
    roads = [];
  }
  savedJson = JSON.stringify(roads);   // 「未保存」灯的基准，就是刚读进来的这份存档
  activeId = roads.length ? roads[0].id : null;
  resampleAll(); renderSide(); runCheck();
  refreshDeploy();

  // 布局尺寸可能还没定下来：重试直到拿到真实宽高，再居中适配底图
  for (let i = 0; i < 40 && !resize(); i++) {
    await new Promise(r => setTimeout(r, 50));
  }
  fitView();
  initZ();
  set3D(true);      // 纹理层就是 FBX 本身的呈现，默认打开
  new ResizeObserver(() => { if (!resize()) return; }).observe(wrap);
  const sc = (META.fbx || "").split("/").pop();
  // 网格尺寸必须摆在脸上：FBX 躺倒或者被缩放过，图看上去完全正常，
  // 要等到 xodr 导进 UE 才发现。数字来自 scene.glb 的顶点包围盒（axis_box）。
  const bx = META.box ? "，网格 " + META.box.map(v => v.toFixed(1)).join(" × ") + " m" +
    (META.box[2] > Math.min(META.box[0], META.box[1]) ? "（⚠ Z 比水平两轴还大，像是躺倒）" : "") : "";
  status("场景 " + sc + bx + "，" + META.w + "x" + META.h + " px，" + roads.length +
    " 条路已载入。地板覆盖 " + (META.floor_frac * 100).toFixed(1) +
    "%（关掉「显示纹理场景」可看到高度底图）");
}
boot();
