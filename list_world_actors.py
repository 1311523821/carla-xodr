#!/usr/bin/env python3
"""列出运行中的 CARLA 世界里所有 actor 的位姿，用来定位"网格到底在哪、有没有歪"。

  python3 list_world_actors.py [--host 127.0.0.1] [--port 2000]

标定前必看的一项：calibrate_frame 解的模型是 scale·R(yaw)·diag(1,±1,1)+t，
只允许绕 Z 转 + 均匀缩放。关卡里摆的 actor 不走 Python API，UE 也没把它的
Scale3D 暴露出来（实测 attributes 是空的），所以缩放和 roll/pitch 得回编辑器
在 Details 面板里看；这里能给你的是位置、朝向和一张世界清单。
"""
import argparse

import carla

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="127.0.0.1")
ap.add_argument("--port", type=int, default=2000)
args = ap.parse_args()

client = carla.Client(args.host, args.port)
client.set_timeout(15.0)
world = client.get_world()
sp = world.get_settings()
print("服务端 %s:%d  地图 %s  同步模式 %s  fixed_delta_seconds %s" % (
    args.host, args.port, world.get_map().name, sp.synchronous_mode,
    sp.fixed_delta_seconds))
m = world.get_map()
try:
    wps = m.generate_waypoints(2.0)
    xs = [w.transform.location.x for w in wps]
    ys = [w.transform.location.y for w in wps]
    zs = [w.transform.location.z for w in wps]
    print("xodr 拓扑：%d 个 waypoint，范围 x[%.1f, %.1f] y[%.1f, %.1f] z[%.1f, %.1f]" % (
        len(wps), min(xs), max(xs), min(ys), max(ys), min(zs), max(zs)))
except Exception as e:
    print("xodr 拓扑生成失败：%s" % e)

print("\n世界里的 actor：")
print("%-30s %6s  %-30s  %s" % ("type_id", "id", "Location(x,y,z)", "Rotation(r,p,y)"))
for a in sorted(world.get_actors(), key=lambda a: a.type_id):
    t = a.get_transform()
    if a.type_id in ("spectator", "controller.player"):
        continue
    print("%-30s %6d  (%9.2f,%9.2f,%9.2f)  (%5.1f,%5.1f,%6.1f)" % (
        a.type_id[:30], a.id, t.location.x, t.location.y, t.location.z,
        t.rotation.roll, t.rotation.pitch, t.rotation.yaw))
