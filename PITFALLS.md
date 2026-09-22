# 踩坑与实测记录

这不是操作手册，是**为什么长成这样**的记录：每条设计决策后面跟着它为什么这么做、
以及不这么做时的实测数字。用法请看 [README.md](README.md)。

内容基本是"当时踩了才知道"的东西，按主题归档。写代码时不必读，改到相关处再查。

---

## 一、TrafficManager 段错误（退出码 139）

**坑：只改 xodr、不重跑 `make import` 的话，TrafficManager 会段错误（退出码 139）。**

`make import` 除了导 FBX，还会在 `Import.py:587` 用 `carla.Map(...).cook_in_memory_map()`
把 xodr 烘成 `Content/<包>/Maps/<图>/TM/<图>.bin` —— TM 用的稀疏采样表。手工 `cp` 新 xodr
进 `OpenDrive/` 之后这份 bin 还是旧的，而缓存**按路径名命中、不带内容哈希**
（`client/detail/Client.cpp:229-245`，根目录 `$HOME/carlaCache/<版本或commit>/`，可用
环境变量 `CARLA_CACHE_DIR` 改），于是链路变成：

```
TrafficManagerLocal::SetupLocalMap (TrafficManagerLocal.cpp:123-131) 读到旧 bin
  -> InMemoryMap::Load (InMemoryMap.cpp:130) 逐条 GetWaypointXODR(road, lane, s)
     路一拆短，118 条记录里 110 条返回 nullptr（旧 xodr 上 0 条）
  -> SetUpSpatialTree (InMemoryMap.cpp:366-371) 只判了 simple_waypoint != nullptr，
     没判它包着的 waypoint -> SimpleWaypoint.cpp:48 解引用空指针
```

判据：崩在 `client.get_trafficmanager()` 那一刻，而 `manual_control.py` 一切正常
（它不调 TM）。跟端口、跟后台残留进程、跟 `-game` 还是 PIE 都无关。

等价的命令，编辑器没开着时用这个：

```bash
M=<地图名>; PKG=$M; CARLA_ROOT=/path/to/carla
cp cache/$M/out/$M.xodr $CARLA_ROOT/Unreal/CarlaUE4/Content/$PKG/Maps/$M/OpenDrive/$M.xodr
rm -f $CARLA_ROOT/Unreal/CarlaUE4/Content/$PKG/Maps/$M/TM/$M.bin ~/carlaCache/*/$PKG/Maps/$M/TM/$M.bin
#    然后回 UE 重开一次 Play（服务端在加载地图时才解析 xodr，不重启世界里的车道还是旧值）
```

**只有这两条命令是必须的**，`cook_in_memory_map` 是可选优化：不烘的话每个客户端进程自己重建
（日志 `No InMemoryMap cache found. Setting up local map.`），本例 82 个采样点是**瞬间**的事；
只有换成 Town 级的大图（`Town13.bin` 15 MB / 几十万点）才值得烘。

### 采样表有两份，实测矩阵（同一个 xodr，只换表的组合）

| 服务端 `Content/.../TM/` | 客户端 `~/carlaCache/.../TM/` | 结果 |
|---|---|---|
| 旧 | 无 | **139** —— 清单里有，客户端去下载，下到旧的 |
| 无 | 旧 | 正常 —— 清单里没有，客户端那份根本不会被读 |
| 无 | 无 | 正常 —— 打印 `No InMemoryMap cache found` 后运行时重建 |
| 新 | 旧 | **139** —— `Client.cpp:234-240` 见本地已存在就**不重新下载**，直接读旧的 |
| 新 | 新 | 正常 |

所以规则是：**要么服务端根本没有这份表，要么两边都是新的**。`rm -f` 两处是幂等的、
不用判断的、永远正确的做法。

### 什么时候表才会真的失效

bin 里存的只有 `(road_id, lane_id, s)` 三元组（加拓扑/网格编号），判据就是"这三样有没有变"。
改路网点、增删路、改 `link_next` 都会改路长 → 三元组悬空 → 139。
**纯改车道宽度不会**：实测 2.0 与 2.4 两份 xodr 的 `length` 逐位相同
（16.7371008182396075 / 18.4232535621717943），只有 `<width a>` 变了，那份 82 条的表在 2.4 上
依旧 0 条悬空 —— 这种情况连 `rm` 都不用，只有 `cp` + 重开 Play。

### 部署按钮的实测细节

「部署到 CARLA」整个 POST 实测 **4–5 毫秒**（页面内 5 次：5/31/5/5/4 ms），
慢的是重开 Play 那一下，不是它。

静态 JS 是每次现读磁盘的，Python 路由却活在启动那一刻的进程里 —— 旧进程配新按钮，
点下去会 404/405 返回 HTML。所以现在所有接口都先按文本收再解析，失败时明说
"服务端没有这个接口（HTTP 405）—— 编辑器进程是旧的，重启 trace_editor.py 才有"，
不会把状态栏卡在"部署中…"。

---

## 二、帧模型：CARLA 在 xodr → world 之间做一次 y 镜像

```
S  Blender/扫描帧，米，右手 Z-up        —— traces.json 写在这个帧
W  CARLA world 帧，米，左手 +Y=右       —— 车实际跑的帧
X  xodr 声明帧                          —— 发射出去的文件的数值
```

**CARLA 在 X -> W 之间做一次 y 镜像**（`world=(x,-y,z)`、`yaw_world=-hdg`）。
实测依据：`line / arc(±) / paramPoly3(±)` 在该假设下位置残差 0.00000 m，
"无镜像"假设差 8~19 m。注意 `GeometryParser.cpp:117-160` 本身不翻轴，
翻转发生在求值输出边界——**只读解析器源码会得出相反的错误结论**。

车道符号因此**不需要交换**：一次镜像同时作用在位置和法向上互相抵消，
`lane +1` 在两个帧里指同一物理侧（`validate.py` 的"车道横向归属"断言实测残差 0.0000）。

轴搞错的表现是路整体镜像或躺倒，而**图上看着完全正常**，要等到 xodr 导进 UE 才发现。

### frame.json 为什么按场景存

`A_S2W` 是"这一份扫描 -> world"的刚体，全局一份的话，第二个场景标定完会把第一个的解
悄悄覆盖掉，而两边生成的 xodr 都还在磁盘上。所以是 `cache/<场景>/frame.json`。
验收阈值 `acceptance` 缺省值统一在 `fit_geometry.FRAME_ACCEPTANCE`
（以前 calibrate 用 0.10、check_alignment 用 0.15，两边不一致）。

未标定时 `A_S2W = M` ⇒ `T_S2X = I` ⇒ CARLA 版与 ASAM 版数值相同；标定后两者差异
恰好等于实测残差。`T_S2X` 由代码推导，不要手填。

---

## 三、地板 vs 屋顶 / 树冠：射线为什么从下往上打

室内扫描同时采到屋顶和地板，室外扫描有树冠遮挡地面。**朴素的俯视投影
（从上往下取第一个命中）会得到屋顶或树冠，于是路就规划在半空中。**

实测同一批 XY 列：

| 采样方向 | 中位 z | 那是什么 |
|---|---|---|
| 从上往下（朴素俯视） | +1.23 m | 屋顶 |
| 从下往上（本工具） | -1.37 m | 地板 ✅ |

差 2.17 m ≈ 办公室净高。用 `render_section.py` 可以直接看到这条差距。

同一份网格上量的另一个数，说明这根拉条为什么值得做：扫描覆盖的列里
**67.1% 头顶 1 m 以上还有一个面**。也就是说朴素的俯视投影，三分之二的位置
你会把车规划到天花板上去，而且从图上看它和地面长得一样。

**不用面法向筛朝上的面**：SLAM/Poisson 重建的三角形绕序不可信，实测 32.9 万面
只有 306 面能通过 `nz>0.9` 过滤。

垂直净空由第二次射线给出：`obstacle_height()` 从地板 +0.2 m 往上打，命中即"桌上有
东西/有低垂树枝"，这就是 `probe_clearance` 报"净空 0.26 m"的来源 —— 因为
"有地板"不等于"车能过"，桌腿、椅子、纸箱都在地板上，`floor_z` 先撞地板就停了。

### 这套规则的边界

以下情况会选错，需要"地面分层选择"才能根治：多层场景（高架桥、地库顶板、坡道上方的
雨棚——你要的路不是最低面）；地板下方有 SLAM 漂浮噪点时会先撞到噪点；室外无顶场景可改用
`direction="down"`。

**仍然没做的**：把每列的多层表面显式列出来让人点选"哪层是路面"。当前靠"最低面"规则自动
判定，绝大多数场景够用。分层数据（`FloorField.column_hits`，每列最多 8 层）已经在服务端了。

---

## 四、遮挡判据以前是坏的

道路线、车道宽度带、顶点、路名、问题标记**都会被遮挡**，不是永远画在最上层。
判定和显示层完全同源：

    点 (x, y, z) 可见  <=>  z <= cut 且 不存在扫描面满足 z < h <= cut

即"在高度 cut 处水平剖切、从上往下看"。所以阈值抬到屋顶时屋顶压住地面道路、
路整体消失；阈值降到屋顶以下、家具以上时路又露出来。多层高架场景下，桥面的路
会自然遮住桥下的路，反之亦然。

**这条判据以前是坏的，值得记一笔**：路的高程是在精确点上打射线求的，遮挡却去查一张
0.1 m 的预计算网格，两者差 ±0.9 mm，而判据写的是**严格大于**且没有容差 —— 实测 27 个
采样点里 13 个被**自己脚下的地板**判成"遮挡"。表现就是路画到中间莫名其妙断一截，
而且拖动阈值毫无反应（旧版在 1.20 到 -1.35 之间恒为"11 可见 / 16 遮"）。现在遮挡和
高程同源于对查询点的 `column_hits`，那张网格连同 `init_columns()` 一并删掉了。

实测（`road_0001`：26 个控制点，0.5 m 采样出 27 个；地板 z≈-1.39，头顶那层面在
1.21~1.28 m）：

| 阈值 | 可见 | 被更高的面遮住 |
|---|---|---|
| 9.35 / 2.00（屋顶还在） | 0 | 27 |
| 1.20（屋顶削掉，阈值还在桌面之下） | **27** | 0 |
| 1.25（正好卡在 1.21~1.28 那批面之间） | 6 | 21 |
| 1.30（整层顶棚都降到阈值之下） | 0 | 27 |
| -1.45（阈值低于路面自己） | 0 | 0，27 个点全判 `cut` |

滚轮微调按**滚动距离**计价（一格 ≈100px = 0.05 m，单次最多 0.2 m），不是按事件个数
—— 触控板一次惯性滚动会发上百个事件，按个数计价的话一滑阈值就掉好几米（实测 116 个
事件拽下去 5.8 m）。

---

## 五、FBX → GLB 烘焙：几个必须断言的地方

### 节点变换必须恒等

烘焙脚本在导出前必须 `transform_apply(location, rotation, scale)`，导出后还要断言
**GLB 里所有节点变换都是恒等**（`glb_shrink.node_transforms`）。原因是 `export_apply`
只管修改器不管物体变换：在 Blender 里给物体开了 2 倍缩放导出的 FBX，烘出来节点上带着
`scale=[2,2,2]` 而顶点仍是 1 倍 —— three.js 渲染时会应用节点变换，open3d 的
`read_triangle_mesh` **不应用**，于是屏幕上是 60 m、射线场里是 30 m，描线和看图差两倍。
缓存里如果存着这种旧产物，`bake()` 会认出非恒等节点并重烘，不用手动删。

### 纹理必须缩，不能指望 bpy

为什么要烘一步而不是直接把 FBX 丢进浏览器：`test.fbx` 340 MB，内嵌 61 张 8192x8192 PNG。
这是**体积/显存**限制，不是格式限制 —— three.js 的 `FBXLoader` 能读 7700 二进制几何。

纹理缩小由 `glb_shrink.shrink()` 在 Blender 之后做，**不要指望 `bpy` 的 `Image.scale()`**：
它只改内存缓冲，glTF 导出器仍按原始分辨率打包，实测 61 张里 25 张烘出来还是
8192/4096/2048，占 46 MB 里的 36 MB。Chromium 能把这 2.5 GB（含 mipmap 约 3.4 GB）
全传上显存，**Firefox 分配失败就把那些材质画成白色**，俯视图上满是白斑。缩到 1024 之后
GLB 是 14.9 MB、显存约 325 MB，两个浏览器都稳。`shrink()` 结尾会逐张复核，还有超限就直接
报错；缓存若是旧版脚本烘的（纹理没缩过），`bake()` 会就地补缩而不重跑 Blender。

### 只留一份几何

只有这一份几何，所以"看到的"和"吸附到的"天然同帧。这里曾经还并行烘一份 `field.ply`
给射线求交，两份是各自独立抽稀的（老 PLY 329,217 面 vs 这份 FBX 的 255,709 面），
位置会对不上。`open3d` 解析这份 GLB 只要 **0.5s**（缩纹理之后它从 46 MB 变成 14.9 MB，
之前是 9.8s），读 PLY 是 0.0s。

### 载入耗时实测

冷启动实测 **烘焙 GLB 26.8s（Blender 导入 7.2 + 导出 10.4 + 缩纹理 8.5）→ 建射线场
0.5s → 生成底图 0.8s ≈ 28s**；FBX 的 mtime 没变则第一段直接跳过，约 1.5s 起来。

---

## 六、three.js 正交俯视：画布尺寸恒等于视口

**WebGL 画布尺寸恒等于视口**，缩放改的是相机的取景矩形（`visibleRect()` 逐项复刻 2D 侧的
`w2s`，所以两层逐像素对齐，2D 的取点逻辑一行都不用改）。

别把它改回"画布 = 底图尺寸 × 缩放" —— 那样放大 16 倍就是 11600×13408 = 1.55 亿像素的
帧缓冲（约 620 MB）而屏幕上只有 0.9 MP，而且每个滚轮事件都重分配一次，表现为越放越卡。
实测改前 0.6→155 MP 随缩放平方增长，改后恒为 0.89 MP、每帧耗时一条平线。

裁剪用的是 WebGL 原生剖切面 `Plane((0,0,-1), cut)`，`distanceToPoint = cut - z`，
渲染条件 `z <= cut`。

「3D 轨道查看」切到透视相机后 2D 那套 `w2s` 投影和 3D 相机已经不是同一个视角，所以轨道
模式下**隐藏全部 2D 叠加**（道路、顶点、底图），并把 `#cv` 的 `pointer-events` 摘掉让
拖拽落到 `#gl` 上 —— `#cv` 叠在 `#gl` 之上，不摘掉事件 OrbitControls 就收不到拖拽。

---

## 七、OBJ / FBX 导入的轴向坑

坑在 Blender「导入 Wavefront (obj)」对话框的轴向两项。扫描软件出的 `out.obj` 是 Z-up，
而导入器会按你给的 前进轴/向上轴 硬转一次，转错了扫描就整体躺倒，并且这个躺姿会被 FBX
原样带进 UE（UE 里把 Roll 设 -90 能救回来，就是同一个角度）。

实测（某 30×36 m 室内场景，out.obj 有效顶点框 X -11.291..19.008 / Y -31.066..4.868 /
Z -5.555..9.351，Z 跨度 14.907 才是高度）：

| 前进轴 / 向上轴 | 结果 |
|---|---|
| Y / -X（对话框里看到的默认值） | 高度跑到了 X 上，躺倒 |
| 命令行不传参的工厂默认 `forward=-Z up=Y` | X 30.299 Y 14.907 Z 35.934，躺倒 |
| Y / Z | **恒等映射**：逐位与文件相同，`matrix_world` 是单位阵 ✓ |
| Y / -Z | X 和 Z 一起翻（绕 Y 转 180°），天花板朝下。跨度看着"正常"，最容易选错 |

只有 (Y, Z) 这一组是恒等，`blender/export_fbx.py` 里写死了它，并在导入后断言对象变换为
单位阵。配套的另一半在 `prepare_mesh.py`：导出 GLB 时必须 `export_yup=False`，否则
FBX 站好之后躺倒的换成了 GLB（以前两步互相抵消，没人发现）。

纹理是内嵌的（那份 356.8 MB，删掉同级 `.fbm` 仍能读出全部贴图），所以下游把 FBX 软链到
`Import/` 目录时贴图不会掉 —— 外部 `.fbm` 相对路径会失效。

### 放大扫描的后果

`--scale s` 把整个扫描放大 s 倍（室内 30×36 m 摆不下真实路网，实测用过 4 倍）。
缩放会烘进顶点而不是留在对象变换上。放大之后要记住：

- xodr 里的米是"放大后的米"，1 m = 0.25 m 真实建筑；车仍是 1 倍大
- 层高被放大成 ~60 m，`probe_clearance.py` 的 1.5 m 净空判定永远不会触发
- 底图按 20 px/m 出，127×150 m 是 7.6 M 像素，实测 `floor_z` 1.1s +
  `obstacle_height` 7.6s、峰值 RSS 1.49 GB，所以 PPM 不用跟着改

---

## 八、`make import` 的几个硬规则

- CARLA 的 `Util/BuildTools/Import.py:63-68` 按"**同目录同名** `.xodr` + `.fbx`"配对，
  所以地图名跟着 FBX 文件名走，不能写死 `TestMap`。用符号链接不用复制，否则在 Blender 里
  重新导出后，副本会悄悄变成旧数据而 xodr 是新的。
- 投放目录必须是真实目录 —— `Import.py` 用 `os.walk` 扫，而 `os.walk` 默认**不进入符号
  链接的目录**（实测：把整个包目录软链进 `Import/` 会被整个跳过）。文件级链接没问题。
- `Import.py:612-617` 只在 `Import/` 下**一个 .json 都没有**时才扫描 fbx/xodr 配对，
  跑过一次之后残留的 `<package>.json` 会让它直接用旧的、不再看你新放的地图 ——
  换地图或重导前先清掉 `Import/*.json`。
- FBX 导入参数硬编码在 `Import.py:221-237`（`bConvertScene` / `bConvertSceneUnit` /
  `bCombineMeshes` / `bAutoGenerateCollision` 全为 1），不可在编辑器调。
- **没有任何 commandlet 生成 `.umap`**，关卡仍需在编辑器里复制 BaseMap 手工建。
- 导航网格：`build_binary_for_navigation` 需要 `FBX2OBJ`，缺了就是静默 no-op
  （所以 `addOBJ.py` 会留一个 0 字节的 obj，Nav 是空的）。

---

## 九、场景目录与存档

目录名为什么不用文件名：`test.fbx` 和 `model/test.fbx` 会同名撞进一个目录，两份场景
共用一份 GLB 和一份路点存档。存档按场景分开之后，换加载 FBX 不会把上一个场景的线带过来
—— 以前 `traces.json` 是全局一份，就是这么串台过。

早期版本的路点存档确实是全局一份 `traces.json`。加载某场景时若根目录那份的 `scene`
正好是它，就**搬**进场景目录（`shutil.move`，不是复制）—— 复制会造成"删掉 cache
之后下次加载又复活一份"，等于删不掉。

---

## 十、OpenDRIVE / CARLA 语义坑

**`link_next` 之前是坏的**（别再照旧说法写 `<link to>` / `<link from>` —— OpenDRIVE 没有
这两个属性，CARLA 会直接 `unable to parse the OpenDRIVE XML string`）。真实语义在
`MapBuilder.cpp:705-713`：正方向车道取 `road.GetSuccessor()+lane.GetSuccessor()`，
负方向车道取 `road.GetPredecessor()+lane.GetPredecessor()`，所以发射器给每条被连的车道
同时写 `<link><predecessor id="k"/><successor id="k"/></link>`（id 同号），并且 A→B 时
自动给 B 补上 predecessor —— 只声明一边，B 的入口在图上就是悬空的。
实测（`carla.Map` 离线）：单向续接 3 条边、两条路互相成环 4 条边，两个方向都能跨路。

一个 CARLA 自己的坑：`Map::GetNext`（就是 `Waypoint::next()`）里有一段防环代码，后继的
后继若回到自己就返回空 —— 所以两路成环时 `wp.next()` 走不过去，而 `get_topology()` /
Traffic Manager 用的是 `GetSuccessors`，那边是通的。

**CARLA 会解析但丢弃的东西**（别指望写进去就生效）：`<lateralProfile>` 超高/路拱
（`ProfilesParser.cpp:132-142` 调用已注释）、除 `crosswalk` 与名字前缀 `Speed_` /
`Stencil_` 之外的所有 `<objects>`（`ObjectParser.cpp:33-135`）、trafficGroup、
controller 的 name/sequence。`<elevationProfile>` 有效且**缺省会被注入 0**，所以必须显式写。

**车道标线**：运行时 `generate_opendrive_world` 路径不会调 `GenerateLineMarkings`，
`<roadMark>` 只影响语义不影响可见条纹；要可见标线得走编辑器工具 `OpenDriveToMap.cpp:640`。

**信号灯**：`<signal>` 需要 `country="OpenDRIVE"` 才会被 CARLA 的类型表识别。

**xodr 放置路径**：运行时只认 `<MapDir>/OpenDrive/<MapName>.xodr`
（`OpenDrive.cpp:69-90` + `GetFullMapPath` 剥包名）。手工放的话形如
`Unreal/CarlaUE4/Content/<包名>/Maps/<地图名>/OpenDrive/<地图名>.xodr`。
`Content/Carla/Maps/OpenDrive/` 那个兜底路径只在编辑器查找里，运行时不走。

---

## 十一、环境约束

- **matplotlib 出图中文是方框**：不是缺字体（`fc-list :lang=zh` 有 89 个），而是默认
  `font.sans-serif` 里没有 CJK 家族，回退到了不含中文的 DejaVu Sans。用
  `cnplot.use_chinese()` 一行解决，同时关掉 `axes.unicode_minus` 否则负号也变方框。
- **numpy 锁 1.21.5**，升级会破坏 CARLA 绑定（升到 2.x 后 `import carla` 直接失败；
  open3d 0.17 的 `RaycastingScene` 在 1.21 上可用）。详见 `requirements.txt`。
- **Blender 经常不在 PATH 里**：用绝对路径，别 `apt install blender` —— 发行版仓库那份
  通常落后好几个大版本，而 PLY/OBJ 的导入操作符在不同大版本之间改过名。
- Blender 5.2.2：PLY/OBJ 导入是 `bpy.ops.wm.ply_import`（旧的 `import_mesh.ply` 已移除）；
  `scene.ray_cast(depsgraph, origin, direction)` 首参是 depsgraph。

---

## 十二、其它设计取舍

- **几何判定全在服务端复用既有模块**（`mesh_field` / `probe_clearance` / `fit_geometry`），
  JS 只产生 x,y —— 不在浏览器里重写一套几何逻辑，否则两套实现迟早不一致，那类 bug 最难查。
- **没有地理基准**：这套流程的输入是纯 SLAM 轨迹（扫描成果里没有 GPS 控制点），所以两份
  xodr 都不写 `<geoReference>`。后果：`transform_to_geolocation()` 与 GNSS 传感器数值
  无地理意义。要补就在现场测一个控制点填进 `frame.json`，不要静默写假坐标。
- **路口 junction 故意没做**：多条路汇入同一目标需要 `<junction>`，当前发射器遇到会直接
  报错。`traces.json` 已预留 `junction_in / junction_out` 字段。
- **双向车道的环会互相卡死**：TM 的碰撞半径是 20–33 m（`Constants.h:82-84`，
  `COLLISION_RADIUS_MIN=20` / `RATE=2.65` / `COLLISION_RADIUS_STOP=8`），周长 36.75 m 的
  环上任意两车永远在彼此的判定范围内，对向相遇就双双刹停。要么放大环，要么改单向路。
- `vehicle.bh.crossbike` 是**车**不是行人（要行人用 `walker.*`）。
