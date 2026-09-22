"""把 GLB 里超过上限的纹理缩小重编码，就地重写文件。

为什么不在 Blender 里做：`bpy` 的 `Image.scale()` 只改内存缓冲，glTF 导出器仍旧按
**原始分辨率**打包 —— 实测 61 张里 25 张烘出来还是 8192/4096/2048，占 46 MB 里的
36 MB。Chromium 能把这 2.5 GB（算 mipmap 3.4 GB）纹理全传上显存，Firefox 的 WebGL
分配失败就把那些材质画成白色，于是俯视图上全是白斑。缩放放到这边用 Pillow 做，
做完还能逐张复核，不依赖导出器的行为。
"""
import io
import json
import struct

# PIL 只在动纹理的那两个函数里用。放到模块级会让 Blender 的 python 导不进这个
# 模块（它没有 Pillow），而 prepare_mesh.py 要用下面的 axis_box 做同帧校验。

JSON_CHUNK, BIN_CHUNK = 0x4E4F534A, 0x004E4942
GLB_MAGIC = 0x46546C67


def _read(path, need_bin=True):
    with open(path, "rb") as f:
        magic, ver, total = struct.unpack("<III", f.read(12))
        if magic != GLB_MAGIC or ver != 2:
            raise SystemExit("%s 不是 glTF 2.0 二进制" % path)
        ln, ctype = struct.unpack("<II", f.read(8))
        if ctype != JSON_CHUNK:
            raise SystemExit("%s 第一个 chunk 不是 JSON" % path)
        js = json.loads(f.read(ln))
        if not need_bin:
            return js, b""
        blob = b""
        while f.tell() < total:
            ln2, ctype2 = struct.unpack("<II", f.read(8))
            data = f.read(ln2)
            if ctype2 == BIN_CHUNK:
                blob = data
    return js, blob


def _view_bytes(js, blob, idx):
    v = js["bufferViews"][idx]
    off = v.get("byteOffset", 0)
    return blob[off:off + v["byteLength"]]


def _write(path, js, chunks):
    """chunks 与 bufferViews 一一对应；按序重排 BIN，4 字节对齐补在对应块之后。"""
    out, off = [], 0
    for v, data in zip(js["bufferViews"], chunks):
        v["byteOffset"] = off
        v["byteLength"] = len(data)
        out.append(data)
        off += len(data)
        pad = (-off) % 4
        if pad:
            out.append(b"\x00" * pad)
            off += pad
    js["buffers"][0]["byteLength"] = off
    binchunk = b"".join(out)
    jb = json.dumps(js, separators=(",", ":")).encode()
    jb += b" " * ((-len(jb)) % 4)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", GLB_MAGIC, 2, 12 + 8 + len(jb) + 8 + len(binchunk)))
        f.write(struct.pack("<II", len(jb), JSON_CHUNK))
        f.write(jb)
        f.write(struct.pack("<II", len(binchunk), BIN_CHUNK))
        f.write(binchunk)


def image_sizes(path):
    """GLB 里每张位图的尺寸，供核对。"""
    from PIL import Image, UnidentifiedImageError
    js, blob = _read(path)
    out = []
    for im in js.get("images", []):
        data = _view_bytes(js, blob, im["bufferView"])
        try:
            out.append(Image.open(io.BytesIO(data)).size)
        except (UnidentifiedImageError, OSError):
            continue
    return out


def shrink(path, max_side=1024, quality=85):
    """原地重写 GLB，返回 (缩小张数, 纹理字节 前/后)。缩小后仍有超限就报错。"""
    from PIL import Image, UnidentifiedImageError
    js, blob = _read(path)
    img_views = {im["bufferView"] for im in js.get("images", [])}
    chunks, done, before, after = [], 0, 0, 0
    for i, v in enumerate(js["bufferViews"]):
        data = _view_bytes(js, blob, i)
        if i in img_views:
            before += len(data)
            try:
                im = Image.open(io.BytesIO(data))
                w, h = im.size
                if max(w, h) > max_side:
                    s = max_side / float(max(w, h))
                    im = im.convert("RGB").resize(
                        (max(1, int(w * s)), max(1, int(h * s))), Image.LANCZOS)
                    buf = io.BytesIO()
                    im.save(buf, "JPEG", quality=quality, optimize=True)
                    data = buf.getvalue()
                    done += 1
            except (UnidentifiedImageError, OSError):
                pass                       # 不是位图的 image 引用：原样留着
            after += len(data)
        chunks.append(data)
    _write(path, js, chunks)
    left = [s for s in image_sizes(path) if max(s) > max_side]
    if left:
        raise SystemExit("缩小后仍有 %d 张超过 %d px，最大 %s" % (
            len(left), max_side, max(max(s) for s in left)))
    return done, before, after


IDENT = {"translation": [0.0, 0.0, 0.0], "rotation": [0.0, 0.0, 0.0, 1.0],
         "scale": [1.0, 1.0, 1.0]}


def node_transforms(path):
    """GLB 里所有非恒等的节点变换。

    烘焙产物必须一个都没有：three.js 渲染时会应用节点变换，而 open3d 的
    `read_triangle_mesh` 只读 accessor 里的原始顶点、**不应用**节点变换。节点上
    留个 scale=[2,2,2]，屏幕上就是 60 m、射线场里就是 30 m，描线和看图差两倍。
    """
    js, _ = _read(path, need_bin=False)
    bad = []
    for n in js.get("nodes", []):
        for key, ident in IDENT.items():
            v = n.get(key)
            if v is not None and [round(float(x), 6) for x in v] != ident:
                bad.append((n.get("name") or "?", key, [round(float(x), 4) for x in v]))
    return bad


def axis_box(path):
    """GLB **顶点缓冲自己**的 AABB：只统计 POSITION 访问器，完全不看节点变换。

    这就是 open3d 拿到的坐标，也就是射线求交、地板高度、描线吸附用的那一帧。
    用它对账是因为轴换算出错时文件照样能打开、贴图照样正常，只有整体转了 90°，
    从预览上很难看出来 —— 只能量数字。
    """
    js, _ = _read(path, need_bin=False)
    mn, mx, n = [1e9] * 3, [-1e9] * 3, 0
    for mesh in js.get("meshes", []):
        for prim in mesh.get("primitives", []):
            a = js["accessors"][prim["attributes"]["POSITION"]]
            if a.get("type") != "VEC3" or "min" not in a:
                continue
            for i in range(3):
                mn[i] = min(mn[i], a["min"][i])
                mx[i] = max(mx[i], a["max"][i])
            n += 1
    if not n:
        raise SystemExit("%s 里没有带 min/max 的 POSITION 访问器" % path)
    return mn, mx
