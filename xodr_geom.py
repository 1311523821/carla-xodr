"""解析几何求值，逐式镜像 CARLA 的 LibCarla road/element/Geometry.cpp。

必须与 CARLA 一致而不是与 ASAM 文档一致。实测（tmp/mirror_probe.py，line/arc±/
paramPoly3± 位置残差均为 0.00000 m）确立的帧关系是：

    CARLA world (x, y, z) = (xodr x, -xodr y, xodr z)，   yaw_world = -hdg_xodr

即解析器本身不翻轴（GeometryParser.cpp:117-160 原样传参），但**求值输出到 world 时
有一次 y 镜像**——xodr 帧实际是右手 ASAM 帧，world 是左手帧。

车道符号：xodr 里 lane +1 在 +t（ASAM 左侧）；映射到 world 后仍落在
lateral_normal(yaw_world) = (sin yaw, -cos yaw) 一侧，也就是 world 里的"左"。
一次镜像同时作用在位置和法向上互相抵消，所以 **lane id 与左右宽度在两个帧之间不交换**
（validate.py 的"车道横向归属"断言实测残差 0.0000）。
"""

import math

# xodr 帧 -> CARLA world 帧：y 取反（实测确立，见模块 docstring）
XODR2WORLD = ((1.0, 0.0, 0.0), (0.0, -1.0, 0.0), (0.0, 0.0, 1.0))


def xodr_to_world(x, y, z=0.0):
    return x, -y, z


# Geometry.cpp:141 / :196 —— CARLA 预计算表的采样步长（米）
POLY3_TABLE_STEP = 0.3
PARAMPOLY3_TABLE_STEP = 0.5
PARAMPOLY3_MIN_INTERVALS = 5


def rotate(angle, x, y):
    """Geometry.cpp:61-67 RotatebyAngle."""
    c, s = math.cos(angle), math.sin(angle)
    return x * c - y * s, y * c + x * s


def invert_rotate(angle, dx, dy):
    """rotate() 的逆：world 增量 -> 参考线局部 (u, v)。"""
    c, s = math.cos(angle), math.sin(angle)
    return dx * c + dy * s, dy * c - dx * s


def lateral_normal(hdg):
    """Geometry.cpp:30-31 车道横向法向（+t 方向）。"""
    return math.sin(hdg), -math.cos(hdg)


class Poly3:
    """a + b*p + c*p^2 + d*p^3，含 CARLA 的 Tangent() 一阶导。"""

    __slots__ = ("a", "b", "c", "d")

    def __init__(self, a, b, c, d):
        self.a, self.b, self.c, self.d = float(a), float(b), float(c), float(d)

    def evaluate(self, p):
        return self.a + p * (self.b + p * (self.c + p * self.d))

    def tangent(self, p):
        return self.b + p * (2.0 * self.c + 3.0 * self.d * p)


class DirectedPoint:
    __slots__ = ("x", "y", "hdg")

    def __init__(self, x, y, hdg):
        self.x, self.y, self.hdg = float(x), float(y), float(hdg)

    def __repr__(self):
        return "DP(%.4f, %.4f, %.5f)" % (self.x, self.y, self.hdg)


class Geometry:
    """参考线的一段。kind ∈ {line, arc, paramPoly3}。"""

    __slots__ = ("s", "x", "y", "hdg", "length", "kind", "curvature", "pu", "pv",
                 "arc_length", "_table")

    def __init__(self, s, x, y, hdg, length, kind, curvature=0.0, pu=None, pv=None,
                 arc_length=True):
        self.s, self.x, self.y, self.hdg, self.length = s, x, y, hdg, length
        self.kind = kind
        self.curvature = curvature
        self.pu, self.pv = pu, pv
        self.arc_length = arc_length
        if kind == "paramPoly3":
            if pu is None or pv is None:
                raise ValueError("paramPoly3 必须给 pu/pv 多项式")
            self._table = self._precompute()

    # ---- 起点/终点 -------------------------------------------------------
    def start(self):
        return DirectedPoint(self.x, self.y, self.hdg)

    def end(self):
        return self.at(self.length)

    # ---- 求值 ------------------------------------------------------------
    def at(self, dist):
        """局部弧长 dist -> DirectedPoint（位置 + 朝向）。"""
        d = min(max(dist, 0.0), self.length)
        if self.kind == "line":
            lx, ly = d, 0.0
            hdg = 0.0                     # 相对量：直线不改变朝向
        elif self.kind == "arc":
            lx, ly, hdg = self._arc(d)
        elif self.kind == "paramPoly3":
            lx, ly, hdg = self._param(d)
        else:
            raise ValueError(self.kind)
        wx, wy = rotate(self.hdg, lx, ly)
        return DirectedPoint(self.x + wx, self.y + wy, self.hdg + hdg)

    def _arc(self, d):
        """Geometry.cpp:45-58 GeometryArc::PosFromDist。"""
        k = self.curvature
        if abs(k) < 1e-15:
            return d, 0.0, 0.0
        r = 1.0 / k
        h = math.pi / 2.0
        x = r * math.cos(h)
        y = r * math.sin(h)
        tangent = d * k
        x2 = r * math.cos(h + tangent)
        y2 = r * math.sin(h + tangent)
        return x - x2, y - y2, tangent

    def _param(self, d):
        """Geometry.cpp:196-214 + PreComputeSpline 的建表线性插值。"""
        table = self._table
        if d <= table[0][0]:
            i = 0
        elif d >= table[-1][0]:
            i = len(table) - 2
        else:
            lo, hi = 0, len(table) - 1
            while hi - lo > 1:
                mid = (lo + hi) // 2
                if table[mid][0] <= d:
                    lo = mid
                else:
                    hi = mid
            i = lo
        s1, u1, v1, tu1, tv1 = table[i]
        s2, u2, v2, tu2, tv2 = table[i + 1]
        rate = 1.0 if s2 == s1 else (s2 - d) / (s2 - s1)
        u = rate * u1 + (1.0 - rate) * u2
        v = rate * v1 + (1.0 - rate) * v2
        tu = rate * tu1 + (1.0 - rate) * tu2
        tv = rate * tv1 + (1.0 - rate) * tv2
        return u, v, math.atan2(tv, tu)

    def _precompute(self):
        """镜像 GeometryParamPoly3::PreComputeSpline（弦长累加，非解析弧长）。"""
        n = max(int(self.length / PARAMPOLY3_TABLE_STEP), PARAMPOLY3_MIN_INTERVALS)
        delta = (1.0 / n) * (self.length if self.arc_length else 1.0)
        rows = []
        p = 0.0
        s = 0.0
        lu, lv = self.pu.evaluate(p), self.pv.evaluate(p)
        rows.append((s, lu, lv, self.pu.tangent(p), self.pv.tangent(p)))
        for _ in range(n):
            p += delta
            cu, cv = self.pu.evaluate(p), self.pv.evaluate(p)
            s += math.hypot(cu - lu, cv - lv)
            rows.append((s, cu, cv, self.pu.tangent(p), self.pv.tangent(p)))
            lu, lv = cu, cv
        return rows

    def sampled_table_length(self):
        return self._table[-1][0] if self.kind == "paramPoly3" else self.length

    # ---- XML -------------------------------------------------------------
    def to_xml_children(self):
        g = '<geometry s="%.18g" x="%.18g" y="%.18g" hdg="%.18g" length="%.18g">' % (
            self.s, self.x, self.y, self.hdg, self.length)
        if self.kind == "line":
            return g + "<line/></geometry>"
        if self.kind == "arc":
            return g + '<arc curvature="%.18g"/></geometry>' % self.curvature
        return g + ('<paramPoly3 aU="%.18g" bU="%.18g" cU="%.18g" dU="%.18g" '
                    'aV="%.18g" bV="%.18g" cV="%.18g" dV="%.18g" '
                    'pRange="%s"/></geometry>' % (
                        self.pu.a, self.pu.b, self.pu.c, self.pu.d,
                        self.pv.a, self.pv.b, self.pv.c, self.pv.d,
                        "arcLength" if self.arc_length else "normalized"))


def chain(geoms, step=0.5):
    """把几何链稠密采样成 [(s, x, y, hdg)]，用于与原始折线比对。"""
    out = []
    for g in geoms:
        n = max(int(math.ceil(g.length / step)), 1)
        for i in range(n + 1):
            if i == 0 and out and abs(out[-1][0] - g.s) < 1e-9:
                continue
            d = g.length * i / n
            p = g.at(d)
            out.append((g.s + d, p.x, p.y, p.hdg))
    return out


def polyline_length(pts):
    return sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
               for i in range(len(pts) - 1))
