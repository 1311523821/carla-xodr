# carla-xodr

把扫描成果（FBX）里手工描的道路中心线，变成 CARLA 能解析、Autoware 也能用的
ASAM OpenDRIVE 1.4 文件，并量化验证它贴在扫描路面上。

```
扫描 FBX ──> 浏览器描线 ──> traces.json ──> 拟合 ──> <地图名>.xodr + <地图名>.fbx
                                                    │
                                     校验 / 对齐检查 / 净空探测 / 出图
                                                    │
                                          部署进 CARLA，或 make import
```

设计相关的实测记录（为什么这么做、不这么做会怎样）单独放在
**[PITFALLS.md](PITFALLS.md)**，这份文档只讲怎么用。

---

## 安装

```bash
pip install -r requirements.txt
export BLENDER=/opt/blender/blender        # 改成你的 Blender 可执行文件路径
```

- **Blender**（3.6+ / 4.x / 5.x）只负责 FBX → GLB 的格式转换，不参与几何计算。
  路径可以设 `$BLENDER`，也可以事后在编辑器「⚙ 路径」面板里填（面板优先）。
  不装也能用：只要 `cache/<场景>/scene.glb` 已存在，加载会走缓存。
- **可选 `carla` PythonAPI**：装了之后 `validate.py` 用 CARLA 自己的解析器做离线闭环
  校验（不需要启动服务端），这是最有价值的一组检查；不装则跳过那几项，其余照常。

  ```bash
  pip install carla==0.9.16                # 版本按你的 CARLA 选
  ```

---

## 浏览器描线编辑器

这是主流程：FBX 进，xodr 出，中间不用开第二个终端。

```bash
export BLENDER=/opt/blender/blender
python3 trace_editor.py --port 8071
# 浏览器打开 http://127.0.0.1:8071
```

场景在界面的「加载数据」对话框里选，不在命令行里给 —— 那个 FBX 是这个工具唯一的
场景输入。对话框默认从**家目录**开始翻，一路点进你的扫描目录即可；`--scan-root`
只在扫描放别的挂载点上时才需要（`--scan-root /data/scans`），它圈定对话框能浏览的
范围，服务端会拒绝该目录之外的任何路径。

每个场景一个目录：`cache/<相对扫描根的路径，/ 换成 __>/`，里面是这份场景的全部东西 ——
`scene.glb`（浏览器显示它，服务端的地板高度场和遮挡判定读同一个文件）、`basemap.png`、
这个场景自己的 `traces.json`、`frame.json` 和 `out/`。例：
`<项目>/test.fbx` → `cache/<项目>__test/`。

### 操作流程

1. **加载数据** —— 对话框里翻目录、选中一个 `.fbx`、点「载入所选」。这一步是后台跑的，
   对话框按阶段实时报进度和每段耗时（第一次约半分钟，FBX 没换过则一两秒）。
   载入后画布上就是 FBX 自带的照片纹理，不是高度图。
2. **拖裁剪拉条剥掉遮挡** —— 画布左缘那根标尺，顶端 = 场景 Z 最大值，底端 = Z 最小值，
   往下拖依次裁掉屋顶、桌面/隔断顶，最后只剩地面。帽上的蓝色数字是当前阈值。
   滚轮在标尺上微调，顶栏「裁剪高于」的数值框也可以直接输入。
   取消勾选「裁剪高于」= 完全不裁（阈值数值保留，勾回来即恢复）。
3. **新建路 + 描点** —— 点「＋ 新建路」，然后在画布上左键逐个点。密点要先放大再点：
   状态栏右侧实时显示当前 **px/m 与吸附半径**（顶点吸附是 6 屏幕像素，缩得越小吸附的
   世界距离越大）。
4. **改属性** —— 右侧「当前路属性」填 `width_left` / `width_right` / `speed_kmh` /
   `link_next`（终点接哪条路的 id，留空 = 断头路）。
5. **看实时诊断** —— 每次改动后自动发给服务端跑一遍（约 0.55 s 去抖），侧栏「实时诊断」
   给出每条路的地板覆盖率、最差净空、拟合残差、Δz、几何段构成；「道路列表」右侧的
   徽章同步显示 通畅 / 需调整 / 错误。画布上被更高的面压住的那段路会淡掉，
   顶栏「显示被遮挡的路」可把被遮部分半透明画出来。
6. **保存 traces.json** —— 写进该场景目录下，与 Blender 导出的 schema 兼容。
7. **生成 xodr** —— 当场跑完 拟合 → 发射 → 校验，状态栏报"校验 N 项全过"，
   有 FAIL 则那行转红并列出条目。产物在 `cache/<场景>/out/`，同时自动投放一份
   同名配对的 `xodr` + `fbx` 到 CARLA 的 `Import/` 目录（`--import-dir` 改路径）。
8. **对齐检查** —— 把 xodr 的路面高度逆映射回扫描帧，和网格上的地板逐点比 Δz，
   结果表在侧栏「对齐检查」。约 1~3 秒，不需要 CARLA 服务端。这是"车会不会悬空/
   陷进去"的预测量。
9. **首次导入 vs 之后重新部署**
   - 第一次（或 FBX 换了）：点「⎘ make import」复制命令，拿到终端自己跑。
   - 之后只改了描线/宽度：点「部署到 CARLA」即可，它 = 拷贝 + md5 核对 + 清掉陈旧
     采样表缓存。按钮前面的灯和文字会自己说状态：灰 = 还没 make import，
     橙 = Content 里那份已过期，绿 = 一致（此时按钮是「重新部署到 CARLA」）。
10. **回 UE 重开一次 Play** —— 服务端在加载地图时才解析 xodr，不重启世界里的车道还是旧值。

「⚙ 路径」把部署用的路径摊开给你改，存 `deploy_config.json`（已在 `.gitignore` 里），
**保存后即时生效，不用重启服务端**。

---

## 换一台机器要改的东西

凡是"换个人/换台机器就不成立"的值，一律不在源码里写死，都在「⚙ 路径」面板里，留空即用默认值：

| 字段 | 默认值 | 什么时候要改 |
|---|---|---|
| CARLA 根目录 | `~/carla` | CARLA 不在这个位置 |
| 投放目录 | `<CARLA 根目录>/Import` | 你的 `make import` 扫别的目录 |
| 关卡包名 | 空 = 跟地图名同名 | 关卡放在独立包里 |
| 客户端缓存根 | `$CARLA_CACHE_DIR` 或 `~/carlaCache` | 设过环境变量或换过位置 |
| 服务端地址 / 端口 | `localhost` / `2000` | CARLA 不在同一台机器上。「⎘ 标定」复制出来的命令会带上这两个值 |
| Blender 可执行文件 | 空 = 依次试 `$BLENDER`、`which blender`、`~/blender-*-linux-x64/` | 装在别处且不在 PATH 里 |
| 纹理上限 | `1024` | 显存小就调低，源纹理清晰且显存大可调高 |

面板底部实时显示**解析结果**（会部署到哪个文件、配对投到哪个目录、Blender 找没找到），
填错了当场能看见；保存前逐项校验（端口范围、2 的幂、目录/文件是否存在），
不合法直接红字退回，不会写坏配置。启动时服务端也会把 Blender 和投放目录的实际结果打出来。

**代码里刻意没做成可配置的**：净空判定 `CLEAR_MIN=1.5 m`、平整容差 `FLAT_TOL`、底图分辨率
`PPM=20`、每列表面层数 `COL_LAYERS`、拟合平滑上限、对齐验收阈值。这些一改，「通畅 / 需调整」
和对齐的「通过 / 未通过」的**判定基准**就变了，两个场景之间的数字不再可比 —— 而这套工具的
价值恰恰在数字可比。真要按场景调，改 `trace_editor.py` 顶部那几个常量，并记住之后的
诊断结论不能和之前的对照。

### 一个代码改不掉的坑：Python 版本

`requirements.txt` 把 numpy 锁在 **1.21.5**（CARLA 的 PythonAPI 与 numpy 2.x 的 ABI 不兼容，
实测装上 2.x 后 `import carla` 直接失败），而 numpy 1.21.5 只提供到 **Python 3.10** 的轮子。
所以新机器上要用 `carla` 闭环校验和 open3d 这套，就得用 **Python 3.10**，别用 3.11+，
否则 `pip install -r requirements.txt` 会试图从源码编译 numpy 并失败。

```bash
python3 --version           # 需要 3.10
```

不装 `carla` 也能跑大部分流程（`validate.py` 会跳过 carla 相关项，其余检查照常），
但那时 numpy/open3d 仍是硬依赖，Python 版本这条限制照样在。

### 界面分区

| 位置 | 内容 |
|---|---|
| 工具栏 `01 场景` | 加载数据 |
| 工具栏 `02 描线` | ＋新建路 · 删除当前路 · 撤销 |
| 工具栏 `03 产出` | **每次**：保存 traces.json · 生成 xodr · 对齐检查 · 部署到 CARLA ｜ **首次**：⎘ make import · ⎘ 标定 · ⚙ 路径 |
| 工具栏 `视图` | 裁剪高于 · 车道宽度预览 / 米网格 / 显示被遮挡的路 · 显示纹理场景 / 3D 轨道查看 |
| 画布左缘 | 裁剪 Z 标尺（帽 = 当前阈值读数，尺 = 拖动/滚轮） |
| 右侧栏 | 道路列表 · 当前路属性 · 部署设置 · 实时诊断 · 对齐检查 · 操作提示 |
| 底部状态栏 | 左：未保存灯、纹理载入进度 ｜ 中：最近一次动作的结果 ｜ 右：光标坐标、px/m、吸附半径 |

### 鼠标与键盘

| 操作 | 效果 |
|---|---|
| 左键 空白处 | 给当前路加点（自动吸附扫描网格顶点） |
| Shift + 左键 | 强制加点，不吸附 |
| 左键 已有顶点 | 选中，可拖动 |
| Alt + 左键 线段上 | 在该段插入一个点 |
| 滚轮 | 缩放（在标尺上是微调阈值） |
| 中键 / 右键拖拽 | 平移 |
| `Delete` | 删除选中点 |
| `Ctrl+Z` | 撤销 |

关掉「显示纹理场景」会退回高度底图，配色含义：**青 = 地板，暗 = 无地板，
橙 = 净空不足**。道路线、车道带、顶点、路名都遵守当前剖切阈值，不是永远画在最上层。

「3D 轨道查看」只用于观察：切到透视相机后 2D 叠加会全部隐藏，描线请切回俯视。

---

## 命令行

编辑器里的按钮和这些命令跑的是同一份实现，输出逐字相同。不想开浏览器、或者要在
脚本/CI 里用时走命令行。

### 用合成数据自测（不需要扫描、Blender、CARLA）

```bash
python3 make_demo_trace.py traces.json     # 合成三条路：直 / 圆弧 / S 形
python3 fit_geometry.py --out-dir out
python3 emit_xodr.py --in-dir out --out-dir out --map-name demo
python3 validate.py --in-dir out           # 35~71 项检查，含 CARLA 真解析器闭环
```

装了 `carla` 时最后一步是 70 项（两份 xodr 都过一遍真解析器），没装则跳过 carla 相关项。

### 用真实扫描数据

```bash
export BLENDER=/opt/blender/blender

# 0) 手上还没有 FBX 的话，从扫描工具的 out.obj 转一份（别在 GUI 里随手导入导出）
$BLENDER --background --python blender/export_fbx.py -- \
    --obj ~/scans/<项目>/model/out.obj --out ~/scans/<项目>/<地图名>.fbx
#    --scale s 可把整个扫描放大 s 倍（室内小场景摆不下真实路网时用）

# 1) 描线：走上面的浏览器编辑器
python3 trace_editor.py --port 8071
MESH=cache/<场景目录名>/scene.glb
TRACES=cache/<场景目录名>/traces.json
OUT=cache/<场景目录名>/out
MAP=<地图名>          # = FBX 文件名去掉 .fbx；xodr 和配对的 fbx 都按它命名
#    凡是要 --mesh 的地方都填 scene.glb：它就是浏览器显示的那份几何，
#    描线吸附和看图必然同帧。别另找一份 PLY 顶上，那是另一套抽稀，位置会对不上。

# 1') 也可以在 Blender 里描，结果同样写进 traces.json：
$BLENDER --background --python blender/setup_trace_scene.py -- --mesh $MESH --out trace.blend
#    打开 trace.blend：一条路一条 POLY 曲线，对象名 road_NNNN，
#    曲线属性 xodr.id / width_left / width_right / speed_kmh / link_next，只画平面
$BLENDER --background --python blender/export_curves.py -- --blend trace.blend --out $TRACES

# 2) 通道能不能通车（室内必查：地板上有桌椅挡路）
python3 probe_clearance.py --traces $TRACES --mesh $MESH --clearance 1.5

# 2b) 俯视叠图：描的线 / 拟合参考线 / 车道边线 叠在扫描地板高度图上
python3 render_plan.py --mesh $MESH --traces $TRACES --fitted $OUT/roads_fitted_xodr.json \
    --out $OUT/plan.png
#     青=地板 棕=家具 橙叉=净空不足 绿=你描的线 红=拟合参考线 蓝虚线=车道边线

# 2c) 竖直剖面：证明路在地板上而不是屋顶上（室内/有树冠遮挡时必查）
python3 render_section.py --mesh $MESH --traces $TRACES --axis y --at <某条路的y> --out $OUT/section.png

# 3) 拟合 -> 发射 -> 校验 -> 对齐   （= 编辑器的「生成 xodr」+「对齐检查」）
python3 fit_geometry.py --traces $TRACES --out-dir $OUT
python3 emit_xodr.py --in-dir $OUT --out-dir $OUT --map-name $MAP
python3 validate.py --in-dir $OUT
python3 check_alignment.py --mesh $MESH --xodr $OUT/$MAP.xodr

# 4) 标定 Blender帧 -> CARLA world 帧（需要 CARLA 服务端起着，跑 NewMap 关卡）
python3 calibrate_frame.py --mesh $MESH --extent 60 --step 1.0 --write
#    标定后必须重跑第 3 步，Δz 才是纯粹的拟合误差

# 5) 首次导入 CARLA：视觉网格就是加载进来的那份 FBX，不用再导一次
cd $CARLA_ROOT && make import ARGS="--package=$MAP"

# 6) 之后凡是只改了描线/宽度，走这一步就够，不用重新 make import：
#    编辑器点「部署到 CARLA」，或者手工：
PKG=$MAP               # 关卡包名；留空/同名时就是地图名，见编辑器「⚙ 路径」
cp $OUT/$MAP.xodr $CARLA_ROOT/Unreal/CarlaUE4/Content/$PKG/Maps/$MAP/OpenDrive/$MAP.xodr
rm -f $CARLA_ROOT/Unreal/CarlaUE4/Content/$PKG/Maps/$MAP/TM/$MAP.bin \
      ~/carlaCache/*/$PKG/Maps/$MAP/TM/$MAP.bin
#    然后回 UE 重开一次 Play。为什么必须删这两份，见 PITFALLS.md 第一节。
```

### 清空某个场景的路

删 `cache/<场景>/traces.json`，或者在页面里逐条「删除当前路」再点「保存」。

---

## 输出

| 文件 | 帧 | 用途 |
|---|---|---|
| `out/<地图名>.xodr` + `out/<地图名>.fbx`(链接) | X | 给 CARLA：同目录同名才能配对 |
| `out/<地图名>_asam.xodr` | S | 给 Autoware / lanelet2（右手系，ASAM 1.4） |

## 帧与 frame.json

```
S  Blender/扫描帧，米，右手 Z-up     —— traces.json 写在这个帧
W  CARLA world 帧，米，左手 +Y=右    —— 车实际跑的帧
X  xodr 声明帧                       —— 发射出去的文件的数值
```

`cache/<场景>/frame.json` 存 `calibrate_frame.py` 实测出来的 `A_S2W`（S→W），每个场景一份。
发射用的 `T_S2X` 由代码推导，**不要手填**。没标定时两份 xodr 数值相同，标定后差异
等于实测残差。`check_alignment.py`（或编辑器「对齐检查」）会告诉你标定有没有生效。

## 模块

| 文件 | 职责 |
|---|---|
| `xodr_geom.py` | 逐式镜像 CARLA `road/element/Geometry.cpp` 的解析求值，含 paramPoly3 的 0.5 m 弦长建表插值 |
| `fit_geometry.py` | 折线 → line/arc/paramPoly3 链：平滑 → 曲率分段 → 逐级试候选 → 残差超限递归二分 |
| `emit_xodr.py` | lxml 发射，同一代码路径出两份帧 |
| `validate.py` | 结构不变量 + `carla.Map(name, xml)` 离线真解析 + 闭环比对。`run()` 一份，CLI 与编辑器共用 |
| `mesh_field.py` | 扫描网格地板高度场 + 垂直净空探测 |
| `probe_clearance.py` | 通道可通行性：地板存在 / 平整 / 净空 |
| `render_plan.py` | 俯视叠图 |
| `render_section.py` | 竖直剖面：地板线 / 屋顶线 / 描线高程 / 1.3 m 车高 |
| `cnplot.py` | matplotlib 中文字体配置 |
| `trace_editor.py` + `editor_static/` | 浏览器描线编辑器（Flask + canvas + three.js） |
| `calibrate_frame.py` | 射线采样关卡路面 + 允许反射的最小二解 `A_S2W` |
| `check_alignment.py` | xodr 路面 vs 扫描地板的 Δz 量化（全程离线），`run()` 被 CLI 与编辑器共用 |
| `blender/` | FBX 导出、Blender 侧描线场景搭建、曲线导出 |

## 当前限制

- **路口 junction 未实现**：多条路汇入同一目标需要 `<junction>`，发射器遇到会直接报错。
  `traces.json` 已预留 `junction_in / junction_out` 字段。
- **没有地理基准**：输入是纯 SLAM 轨迹，两份 xodr 都不写 `<geoReference>`，
  所以 `transform_to_geolocation()` 与 GNSS 传感器数值无地理意义。
- **多层场景要人工确认**：地板判定用"竖直列里最低的面"，高架桥、地库顶板这类
  你要的路不是最低面的场景会选错，需要配合 `render_section.py` 的剖面人工核对。
- **导航网格需要额外工具**：`make import` 的 Nav 那一步依赖 `FBX2OBJ`，缺了会静默跳过。

---

## 第三方

`editor_static/vendor/` 里是 [three.js](https://threejs.org/) **r160**（MIT，版权头保留在
文件里）及其 `GLTFLoader` / `OrbitControls` / `BufferGeometryUtils` 附加模块。直接 vendor
而不走 CDN，是因为这个工具跑在没有外网的机器上。
