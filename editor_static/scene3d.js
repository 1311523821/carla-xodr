// 带纹理扫描网格的 WebGL 渲染层，叠在 2D 交互画布底下。
//
// 对齐策略：不用在 3D 里重算投影去凑 2D 的像素坐标，而是把 WebGL canvas 用 CSS
// 摆到与底图完全相同的位置和尺寸，再用一个覆盖同样世界范围的 ortho 相机俯视。
// 这样两层天然逐像素对齐，2D 侧的 w2s/点击取点逻辑一行都不用改。
//
// 高度过滤用 three.js 原生剖切面：Plane((0,0,-1), cut) 使 distanceToPoint = cut - z，
// 渲染条件 z <= cut，正是"裁掉高于阈值的"。
import * as THREE from "three";
import { GLTFLoader } from "./vendor/three/GLTFLoader.js";
import { OrbitControls } from "./vendor/three/OrbitControls.js";

const gl = document.getElementById("gl");
const cvEl = document.getElementById("cv");
let renderer = null, scene = null, camera = null, group = null, controls = null;
let meta = null, mesh = null, plane = null, ready = false, orbiting = false;
let view = { s: 1, ox: 0, oy: 0 };   // 与 2D 侧共享的缩放/平移

function ensureRenderer() {
  if (renderer) return;
  renderer = new THREE.WebGLRenderer({ canvas: gl, antialias: true, alpha: true });
  renderer.setClearColor(0x000000, 0);
  renderer.localClippingEnabled = true;      // 不开这个 clippingPlanes 无效
  scene = new THREE.Scene();
  group = new THREE.Group();
  scene.add(group);
}

/* 当前视口看得见的那块矩形，用**底图 canvas 坐标**表示（group 已经把世界坐标
   平移过，所以这里不含 xmin/ymin）。换算逐项对着 editor.js 的 w2s：
     sx = cx*ppm*s + ox        sy = (meta.h-1 - cy*ppm)*s + oy
   两边用同一个映射，两层才逐像素对齐。 */
function visibleRect() {
  const vw = Math.max(1, cvEl.width), vh = Math.max(1, cvEl.height);
  const k = view.s * meta.ppm;
  return {
    vw, vh,
    x0: (0 - view.ox) / k, x1: (vw - view.ox) / k,
    yTop: (meta.h - 1 - (0 - view.oy) / view.s) / meta.ppm,
    yBot: (meta.h - 1 - (vh - view.oy) / view.s) / meta.ppm,
  };
}

function fitCamera() {
  if (!renderer || !meta) return;
  const r = visibleRect();
  // 画布尺寸固定等于视口，动的是取景框。以前按 meta.w*view.s 开画布，放大 16 倍
  // 就是 11600x13408 = 1.55 亿像素的帧缓冲（约 620 MB），而屏幕上只有 0.9 MP，
  // 而且每次滚轮都要重分配一次 —— 这就是越放越卡的全部原因。
  gl.style.left = "0px";
  gl.style.top = "0px";
  gl.style.width = r.vw + "px";
  gl.style.height = r.vh + "px";
  renderer.setPixelRatio(1);
  // 滚轮一次手势要打几十个事件，setSize 即使尺寸没变也会重置 drawing buffer，
  // 所以显式跳过。现在画布尺寸只跟视口走，正常情况下根本不会变。
  if (gl.width !== r.vw || gl.height !== r.vh) renderer.setSize(r.vw, r.vh, false);
  const cx = (r.x0 + r.x1) / 2, cy = (r.yTop + r.yBot) / 2;
  if (orbiting) {
    camera.aspect = r.vw / r.vh;             // 透视相机只改比例，别抢用户转的视角
    camera.updateProjectionMatrix();
    return;
  }
  // 俯视 ortho。注意 left/right/top/bottom 是**相机空间**的边界、以相机位置为中心，
  // 写成 (0, Wm) 会把视野整体偏移半个场景。
  camera = new THREE.OrthographicCamera(
    -(r.x1 - r.x0) / 2, (r.x1 - r.x0) / 2,
    (r.yTop - r.yBot) / 2, -(r.yTop - r.yBot) / 2, 0.01, 200);
  camera.position.set(cx, cy, 60);
  camera.up.set(0, 1, 0);
  camera.lookAt(cx, cy, 0);
  camera.updateProjectionMatrix();
}

export function init(m) {
  meta = m;
  ensureRenderer();
  new GLTFLoader().load("assets/scene.glb", (gltf) => {
    mesh = gltf.scene;
    // 不转坐标：服务端建射线场读的就是这个 GLB 本身，2D 底图和描线吸附用的
    // 世界坐标和它天然同一个帧。这里再转一次就会和描线错开。
    mesh.traverse(o => {
      if (!o.isMesh) return;
      // 换成无光照的 Basic：GLTF 给的是 Standard 材质，没有灯就全黑；
      // 而且看扫描成果本来就该用照片原色，不该有假打光
      const basic = new THREE.MeshBasicMaterial({
        map: o.material.map || null,
        side: THREE.DoubleSide,               // 扫描网格绕序不可靠，双面才不漏
        clippingPlanes: [planeOf()],
      });
      if (o.material.map) o.material.map.colorSpace = THREE.SRGBColorSpace;
      o.material.dispose();
      o.material = basic;
    });
    // 网格在世界坐标里；group 平移使世界 (xmin,ymin) 落在画布原点
    group.position.set(-meta.xmin, -meta.ymin, 0);
    group.add(mesh);
    ready = true;
    fitCamera();
    render();
    if (window.onSceneReady) window.onSceneReady();
  }, (ev) => {
    if (window.sceneProgress) window.sceneProgress(ev);
  }, (err) => { if (window.sceneError) window.sceneError(err); });
}

function planeOf() {
  if (!plane) plane = new THREE.Plane(new THREE.Vector3(0, 0, -1), 1e6);
  return plane;
}

export function setCut(cut) {
  planeOf().constant = cut;                   // distanceToPoint = cut - z，z<=cut 才画
  render();
}

export function setEnabled(on) {
  gl.style.display = on ? "block" : "none";
  if (on) { fitCamera(); render(); }
}

export function render() {
  if (!ready || !renderer || !camera || gl.style.display === "none") return;
  renderer.render(scene, camera);
}

export function setView(v) { view = v; fitCamera(); render(); }

// 轨道查看：切到透视相机自由转，只用于观察场景；退出后回到与 2D 对齐的俯视
export function setOrbit(on) {
  if (!ready) return;
  orbiting = on;
  gl.style.cursor = on ? "grab" : "default";
  if (on) {
    const r = visibleRect();
    const cx = (r.x0 + r.x1) / 2, cy = (r.yTop + r.yBot) / 2;
    const span = Math.max(r.x1 - r.x0, r.yTop - r.yBot);
    camera = new THREE.PerspectiveCamera(50, r.vw / r.vh, 0.1, 500);
    camera.position.set(cx, cy - span * 1.2, span * 0.9);
    if (!controls) {
      controls = new OrbitControls(camera, gl);
      controls.enableDamping = true;
      controls.addEventListener("change", render);
    } else {
      // 每次进轨道模式都会新建一台透视相机，而 OrbitControls 在构造时就把
      // this.object 绑定死了（vendor/three/OrbitControls.js:34），只认第一台。
      // 不重新指过去的话，第二次进轨道拖拽的是那台已经被丢弃的相机 —— 表现为
      // 画面纹丝不动。俯视相机的 fitCamera/computeCamera 不受影响。
      controls.object = camera;
    }
    controls.enabled = true;
    controls.target.set(cx, cy, 0);          // 绕当前看得那块转，而不是绕整张场景
    controls.update();
  } else {
    if (controls) controls.enabled = false;
    fitCamera();
  }
  render();
}
