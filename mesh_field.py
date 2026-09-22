"""扫描网格的"地板"高度场。

不能靠面法向筛朝上：SLAM/Poisson 重建的三角面绕序通常不一致，法向不可信
（实测 32.9 万面只剩 306 面通过 nz 过滤）。改用几何事实：
direction='up' 从场景最低点往**上**打射线，第一个命中就是最低的水平面 = 地板，
天花板因为在它上方而天然被排除。室外无顶场景用 direction='down'。
"""


import numpy as np
import open3d as o3d


class FloorField:
    def __init__(self, mesh_path, direction="up"):
        m = o3d.io.read_triangle_mesh(mesh_path)
        if m.is_empty():
            raise SystemExit("读不到网格: %s" % mesh_path)
        v = np.asarray(m.vertices, dtype=float)
        t = np.asarray(m.triangles, dtype=np.int32)
        if not len(t):
            raise SystemExit("%s 里没有三角面" % mesh_path)
        self.tri_count = len(t)
        self.x_min, self.x_max = float(v[:, 0].min()), float(v[:, 0].max())
        self.y_min, self.y_max = float(v[:, 1].min()), float(v[:, 1].max())
        self.z_min = float(v[:, 2].min())
        self.z_max = float(v[:, 2].max())
        self.direction = direction
        sub = o3d.geometry.TriangleMesh()
        sub.vertices = o3d.utility.Vector3dVector(v)
        sub.triangles = o3d.utility.Vector3iVector(t)
        tm = o3d.t.geometry.TriangleMesh.from_legacy(sub)
        self.scene = o3d.t.geometry.RaycastingScene()
        self.scene.add_triangles(tm)

    def floor_z(self, xy):
        """批量求地板高度。xy: (N,2)。返回 (N,) 未命中为 nan。"""
        q = np.asarray(xy, dtype=np.float32)
        up = self.direction == "up"
        z0 = (self.z_min - 1.0) if up else (self.z_max + 1.0)
        dz = 1.0 if up else -1.0
        org = np.stack([q[:, 0], q[:, 1], np.full(len(q), z0, dtype=np.float32)],
                       axis=1)
        d = np.tile(np.array([0.0, 0.0, dz], dtype=np.float32), (len(q), 1))
        rays = np.concatenate([org, d], axis=1)
        ans = self.scene.cast_rays(o3d.core.Tensor(rays))
        th = ans["t_hit"].numpy().astype(float)
        z = org[:, 2] + th * dz
        z[~np.isfinite(th)] = np.nan
        return z

    def obstacle_height(self, xy, floor_z_vals, probe=0.20, ceiling=3.0):
        """地板之上第一个障碍物的相对高度（米）。

        从 floor+probe 往上打射线：命中即桌上有东西/有椅子挡路，返回命中高度减地板；
        未命中返回 +inf（净空充足）。注意 floor_z 本身探测不到家具，因为它从下往上
        先撞到地板，必须靠这一层才能判断通道能不能通车。
        """
        q = np.asarray(xy, dtype=np.float32)
        fz = np.asarray(floor_z_vals, dtype=np.float32)
        org = np.stack([q[:, 0], q[:, 1], fz + np.float32(probe)], axis=1)
        d = np.tile(np.array([0.0, 0.0, 1.0], dtype=np.float32), (len(q), 1))
        rays = np.concatenate([org, d], axis=1)
        ans = self.scene.cast_rays(o3d.core.Tensor(rays))
        th = ans["t_hit"].numpy().astype(float)
        h = np.full(len(q), np.inf)
        good = np.isfinite(th) & (th < ceiling)
        h[good] = th[good] + probe
        h[~np.isfinite(fz)] = np.nan
        return h

    def column_hits(self, xy, k=8, eps=0.02):
        """一列上从下往上依次命中的前 k 个表面高度，形状 (k, N)，未命中为 nan。

        这是"高度过滤"的数据基础：室内扫描同一列至少有地板和屋顶两个面，
        只取第一个命中（floor_z）拿不到中间的桌面/隔断那一层。
        实测 14.7 万列 x 8 次弹射约 0.1 s，可以直接在启动时全量算好缓存。
        """
        import open3d as o3d
        q = np.asarray(xy, dtype=np.float32)
        z = np.full(len(q), self.z_min - 1.0, dtype=np.float32)
        d = np.tile(np.array([0.0, 0.0, 1.0], dtype=np.float32), (len(q), 1))
        hits = np.full((k, len(q)), np.nan, dtype=np.float32)
        for i in range(k):
            live = np.isfinite(z)
            if not live.any():
                break
            org = np.stack([q[:, 0], q[:, 1], z], axis=1).astype(np.float32)
            rays = np.concatenate([org[live], d[live]], axis=1)
            th = self.scene.cast_rays(o3d.core.Tensor(rays))["t_hit"].numpy() \
                .astype(np.float32)
            hit_z = np.full(len(q), np.nan, dtype=np.float32)
            got = np.isfinite(th)          # th 未命中时是 inf，直接相加会污染成 inf
            idx = np.nonzero(live)[0][got]
            hit_z[idx] = z[idx] + th[got]
            hits[i] = hit_z
            z = np.where(np.isfinite(hit_z), hit_z + eps, np.float32(np.inf))
        return hits


def grid_xy(cx, cy, extent, step):
    n = int(2 * extent / step) + 1
    ax = cx + np.linspace(-extent, extent, n)
    ay = cy + np.linspace(-extent, extent, n)
    gx, gy = np.meshgrid(ax, ay)
    return np.stack([gx.ravel(), gy.ravel()], axis=1)
