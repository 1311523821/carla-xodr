"""把带纹理的 FBX 转成编辑器用的 GLB（你不用手动跑 Blender）。

  blender --background --python prepare_mesh.py -- --fbx <输入.fbx> --glb <scene.glb>

这一步只做格式转换：解 FBX、挑出最大的那个网格、让导出器把内嵌纹理转 JPEG 打包。
**不换轴**：GLB 的顶点缓冲和 FBX 在 Blender 里的世界框逐位相同，末尾会读回 accessor
的 min/max 对账（原因见导出调用处的注释）。
**纹理缩小不在这里**，在服务端的 glb_shrink.shrink() 后处理里做 —— bpy 的
Image.scale() 只改内存缓冲，导出器仍按原始分辨率打包，实测 61 张里 25 张烘出来
还是 8192/4096/2048。

只产一份 GLB：服务端的地板高度场/遮挡判定用 open3d 直接读同一个 GLB（实测 0.5 s）。
曾经还并行烘一份 field.ply 给射线求交，那是两套抽稀、两个坐标系，看图的位置和
吸附的依据会对不上；一份资产就没有"是否同帧"这个问题。
"""
import os
import sys
import time

import bpy


def arg(name, default=None):
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    for i, a in enumerate(argv):
        if a == name and i + 1 < len(argv):
            return argv[i + 1]
    return default


def main():
    fbx = arg("--fbx")
    out = arg("--glb", "scene.glb")
    if not fbx or not os.path.exists(fbx):
        raise SystemExit("必须给 --fbx <存在的 .fbx 路径>")

    bpy.ops.wm.read_factory_settings(use_empty=True)
    t0 = time.time()
    bpy.ops.wm.fbx_import(filepath=fbx)
    meshes = [o for o in bpy.context.scene.objects if o.type == "MESH"]
    if not meshes:
        raise SystemExit("FBX 里没有网格")
    m = max(meshes, key=lambda o: len(o.data.polygons))
    print("导入 %.1fs  网格 %s: 顶点 %d 面 %d UV %d 材质 %d" % (
        time.time() - t0, m.name, len(m.data.vertices), len(m.data.polygons),
        len(m.data.uv_layers), len(m.data.materials)))
    if not m.data.uv_layers:
        raise SystemExit("网格没有 UV，纹理贴不上去")

    for o in bpy.context.scene.objects:
        o.select_set(o is m)
    # 必须把物体变换烘进顶点：导出器的 export_apply 只管修改器，不管物体变换，
    # 带非 1 缩放的 FBX 会原样写成节点 scale。three.js 会应用节点变换而 open3d
    # 不应用 —— 两边就差那个倍数（实测 2x 缩放的 FBX 显示 60 m、射线场 30 m）。
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    # 不要在这里改成 bpy.context.view_layer.objects.active —— transform_apply 不改变
    # 活动物体，而它未必是上面挑出的那个最大网格（active 是导入时最后选中的那个）。
    # 一旦指错，下面按 m.matrix_world 算的世界框量的就是另一个物体，末尾的对账会报
    # "轴换算被改过了"并 exit 1 —— 而 GLB 其实是对的。实测两个网格（active=小的、
    # 大的 96 面）时偏差 4.500 m。当前扫描 FBX 都是单网格所以一直没暴露。m 仍是
    # max(...) 挑出的那个，且已 select_set 单独选中。
    print("烘进顶点 matrix_world=%s" % [round(v, 4) for v in
          [x for row in m.matrix_world for x in row]])
    bmn, bmx = [1e9] * 3, [-1e9] * 3
    for v in m.data.vertices:
        c = m.matrix_world @ v.co
        for i in range(3):
            bmn[i] = min(bmn[i], c[i])
            bmx[i] = max(bmx[i], c[i])
    t1 = time.time()
    # export_yup=False：glTF 规范拿 Y 当高度，导出器默认把 (x,y,z)->(x,z,-y) 烘进顶点。
    # 我们这份 GLB 不是要发布的资产，而是 open3d 射线场和浏览器共用的那一帧，而整套
    # 工具（俯视正交相机、从下往上的射线、z 裁剪拉条）都假定 Z 是高度。
    # 以前两步互相抵消所以没人发现：FBX 本身躺倒，导出器再转一次刚好站起来了。
    # FBX 从源头修好之后必须显式关掉这个转换，否则躺倒的就换成 GLB 了
    # （实测跨度 Z 从 14.907 变成 35.934）。
    bpy.ops.export_scene.gltf(filepath=out, use_selection=True,
                              export_format="GLB", export_image_format="JPEG",
                              export_apply=True, export_materials="EXPORT",
                              export_yup=False)
    mb = os.path.getsize(out) / 1e6
    print("写出 %s  %.1f MB  (%.1fs)" % (out, mb, time.time() - t1))

    # 对账：GLB 顶点缓冲的框必须和 Blender 里的世界框一模一样。
    # 用回读 Blender 场景的办法量不了这件事——glTF 导入器会自作主张转回 Y-up，
    # 量到的是它的猜测而不是文件里的事实，所以直接读 accessor 的 min/max。
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    import glb_shrink
    gmn, gmx = glb_shrink.axis_box(out)
    d = max(max(abs(gmn[i] - bmn[i]), abs(gmx[i] - bmx[i])) for i in range(3))
    if d > 0.01:
        raise SystemExit("GLB 和 FBX 不同帧：Blender 框 %s..%s，GLB 框 %s..%s，差 %.3f m"
                         " —— 轴换算被改过了" % (
                             [round(v, 3) for v in bmn], [round(v, 3) for v in bmx],
                             [round(v, 3) for v in gmn], [round(v, 3) for v in gmx], d))
    print("同帧校验 OK  X %.3f..%.3f  Y %.3f..%.3f  Z %.3f..%.3f（最大偏差 %.4f m）" % (
        gmn[0], gmx[0], gmn[1], gmx[1], gmn[2], gmx[2], d))


main()
