# BEV、Reference 与特殊元素扩展合同

本文定义当前 `new/` 运行时中视觉事实、特殊元素、reference、控制资格和安全门的责任边界。开源使用、任务变化和跨平台迁移从仓库根 [`README.md`](../../README.md) 进入；参数键以 [`../config/default_params.md`](../config/default_params.md) 为准；port 类型和 include/memory ownership 以 [`../code/port/README.md`](../code/port/README.md) 为准。

当前 authority 是 `new/user/CMakeLists.txt:NEW_SRCS` 及其编入的 active source、实际运行参数/资产和同版本证据。固定 commit、archive、superseded 文档、旧 overlay 或历史板测只说明迁移背景。

## 1. 当前数据流

```text
camera frame + capture metadata
-> illumination binary model
-> sparse BEV row/road facts
   -> ordinary line candidate
   -> Cross evidence + Cross straight candidate
   -> Zebra evidence
   -> CircleV2 SceneFrameView
-> ML observer -> optional ML maneuver candidate
-> Zebra stop scene -> optional controlled-stop intent
-> CircleV2 observers -> reducer -> composer -> optional Circle candidate
-> explicit visual candidate policy
-> current visual reference
-> continuity (current / hold / none)
-> usability
-> tracking geometry
-> control readiness
-> time alignment at control tick
-> safety gate
-> motion supervisor
-> yaw target / gyro feedback
-> wheel target mixer / wheel PID
-> command builder / shaper
-> actuator apply observation
-> assistant / steering-media serialization
```

候选不是全局置信度排序。当前 `SteeringFramePipeline` 的政策为：

1. Zebra present：只提交 ordinary line；Zebra 任务通过独立 stop scene 表达。
2. 否则 ML scene active：只提交 ML candidate。
3. 否则 Cross present 且 `BEV_ELEMENT.CROSS_TAKEOVER_ENABLED=1`：只提交 Cross candidate。
4. 其他情况：提交 ordinary line，并按存在性附加 Circle candidate。

新增元素必须显式定义插入哪个分支、与 Zebra/ML/Cross/Circle/ordinary 同时出现时谁拥有任务决策、被压制或禁用时如何复位自己的 memory。不得用偶然置信度竞争代替政策。

## 2. 跨层事实语义

| 字段/类型 | 唯一语义 | 不具有的语义 |
| --- | --- | --- |
| `BEVPathSample::present` | 对应路径样本存在 | 可控制、安全 |
| `BEVPathPointSource` / `ReferenceMode` | 路径来源，仅供解释 | usability、gate 或 yaw 条件 |
| `ReferenceUsability::usable` | leading samples 连续且几何有限 | 传感器和电源健康 |
| `ReferenceTrackingGeometry::computed` | offset/heading/curvature 已计算且有限 | actuator 可输出 |
| `ReferenceLateralErrorEstimate::computed` | legacy weighted lateral-error 对照值可用 | 当前主控输入 |
| `ReferenceControlReadiness::ready` | selected reference + tracking geometry 可进入控制 | safety gate clear |
| `ControlGateDecision::veto_active` | safety owner 禁止控制 | 上游事实为何错误 |
| `PerceptionResult` | 单帧 pipeline 发布的跨线程 snapshot | 可被任意层 include 的依赖捷径 |

`source`、`mode`、`reason`、confidence、overlay、media 和 assistant telemetry 只能解释生产事实，不参与 usability、tracking geometry、readiness、gate 或 yaw 决策。

## 3. 图像采样与 BEV 事实

- 原图提供像素采样值；当前视觉几何事实面是 BEV metric sparse rows。
- `BEV_PROJECTOR` 决定图像到车辆坐标的投影。相机、分辨率、裁剪、镜头或安装姿态变化先重做投影/ROI 标定，不能用 PID、hold、crop 或元素阈值补偿错误几何。
- sample 投影状态只表达 `kSampleable`、`kOutsideFrame` 或 `kProjectionFailed`。FOV 外、投影失败、图像边缘和搜索边界不是 edge、opening、元素或路径 evidence。
- sparse row 由统一 binary model 和 luma sampler 产生 boundary jumps/spans/traces；同一行两侧边点只有形成 boundary span 才属于同一道路片段。
- row 内 connectivity 只判断真实可采样图像段；不可采样区域不能被当作道路隔断或连通证据。
- row 间 boundary continuity 使用 BEV 米制距离；任何 `*_DISTANCE_M` 不能按原图像素解释。
- blocked、不可观测或 empty row 可被跳过以寻找后续真实样本，但 builder 不补点、不插值、不跨 gap 连接。
- full BEV raster 当前不是主程序 Cross、Zebra 或 CircleV2 的公共热路径。新消费者只有确实需要二维邻接事实时才引入局部 ROI/raster，并承担采样有效性、性能和测试合同。

## 4. Reference 合同

- `BEVReferencePath` 的 index 0 锚定近端原点；后续 present 点紧凑排列并保留真实 forward 位置。
- ordinary reference 只能来自通过图像段 connectivity 的 boundary span/trace；单边丢线修复使用有明确几何合同的 signed-normal offset，双边丢失交给 continuity。
- visual orchestration 是 line/element candidates 生成 current visual reference 的唯一 owner。
- continuity 是 hold 的唯一 owner。hold 只复制上一条 selected reference 的 leading present 段，将 source 改为 hold，不覆盖最后视觉事实。
- usability 只检查 leading reference；index 0 断裂时不能用远端点控制。
- tracking geometry 只消费 selected reference + usability，不依赖 speed、source 或 mode。
- readiness 只消费 usability、tracking geometry 和 hold/degraded 事实；low voltage、projector、stale、IMU、encoder 不属于 readiness。
- 初次启动边界不允许 held/degraded reference；运行中的 hold 仍逐层经过 usability、tracking、readiness 和 gate。
- legacy weighted lateral error 只用于迁移对照，不是当前 yaw target 的主控输入。

## 5. 当前特殊元素 owner

### 5.1 Cross

- `new/code/vision/elements/cross_exit_element_evidence.*` 只从 sparse rows/connectivity 产生 Cross evidence。
- `cross_straight_path_planner.*` 独立把 grounded evidence、sparse rows 和 segment connectivity 组合为 straight candidate。
- `visual_element_pipeline.*` 编排 detector、Zebra detector 和 Cross planner，不负责最终候选选择。
- `CROSS_TAKEOVER_ENABLED=0` 只关闭 Cross candidate 接管；检测、evidence 和 telemetry 仍运行。
- Cross detector 不读取 Circle/Zebra/ML scene memory、hold、gate、IMU、encoder、yaw 或 actuator。

### 5.2 Zebra

- `zebra_element_evidence.*` 每帧从 sparse rows 产生 Zebra evidence；当前没有 enabled 参数。
- `zebra_stop_scene.*` 独占首次识别、连续消失、再遇、延时和 stop-requested 状态。
- Zebra present 时 reference policy 保留 ordinary line；`controlled_stop_requested` 经控制侧 adapter 变为 MotionSupervisor 的 stop intent。
- Zebra scene 可以请求受控停车，但不能直接写 PWM；gate、motion lifecycle、command shaping 和 apply observation 仍生效。
- 因为没有 enabled 参数，“完全不识别特殊元素”必须在 pipeline 装配处移除 Zebra producer/scene，不能在控制侧忽略 stop intent 来补偿。

### 5.3 CircleV2

`CircleV2` 是独立 scene，不是旧的 “Phase1 detector → Phase2 ROI candidate builder”：

```text
SceneFrameView(sparse rows, ordinary road, motion arc, capture stamp)
-> entry-cue observer
-> geometry observer
-> event observer
-> reducer updates CircleV2 memory and reference role
-> geometry selection
-> composer
-> optional reference plan
-> reference adapter / connectivity filter
-> visual candidate policy
```

- scene phase 为 `Idle`、`Approach`、`InnerTrace`、`NormalTrace`、`ExitTrace`、`CalmTrace`、`Cooldown`。
- reducer 是状态迁移和 reference role 的 owner；observer 不自行改变状态，composer 不自行决定 phase。
- `BEV_ELEMENT.CIRCLE_V2_ENABLED` 决定每帧是否运行 scene。当前没有独立 Circle takeover/Phase2 开关；禁用时必须复位非 idle memory。
- 当前参数按 enable、entry/opening/geometry、normal/exit/calm yaw、stall 和 cooldown 归组；精确名称只从参数合同、loader 和 serializer 取得，本文不复制值。
- Circle candidate 仍经过 connectivity、visual policy、continuity、usability、tracking、readiness、gate 和控制链。

### 5.4 ML

- ML 不属于 Cross/Zebra `VisualElementPipeline`；`SteeringFramePipeline` 分别调用 ML observer 和 maneuver scene。
- observer 消费有效 `32x32 gray8` ROI，发布 classifier/ROI facts；分类不能直接写路径、速度或 actuator。
- `ML.ENABLED=1`、`ML.MANEUVER.ENABLED=0` 是 observe-only。
- maneuver scene 启用后，才可把已确认 action 与 ordinary road facts、motion history 组合为 grounded candidate。
- 只有 ML candidate 被 visual orchestration 选中后，`takeover_selected` 才允许 speed policy 使用 `ML.MANEUVER.SPEED_TARGET`。
- classifier backend、类映射、连续确认、path generation 和 speed policy 是不同合同；V9 Hamming 与 TFLite identity squared-L2 阈值不得共用。

## 6. 新元素或新任务的扩展规则

1. 先定义可观察事实、单位、invalid/absent 语义和 producer；输入不能区分正常与故障时先扩展 producer validity，不在 detector 中猜。
2. 使用最小输入面：sparse rows 足够时不建 raster；需要局部二维、颜色或 ML 时明确新增 ROI/frame contract。
3. detector 只产生当前帧 evidence，不读 hold、safety、IMU、encoder、actuator 或 yaw memory。
4. 只有路径型元素才实现 candidate builder；builder 不能补造点、跨 gap 或把边界外事实变成 reference。
5. 只有跨帧任务才实现 scene/reducer；必须定义进入、保持、退出、超时、重入、复位和 dropped-frame 语义。
6. 路径型任务进入显式 candidate policy；停车/速度/其他动作由独立 task policy 输出受限 intent，不能由 detector 直接写 PWM。
7. 新能力默认先 evidence/observe-only；candidate 或 action 默认不授权，直到 synthetic owner tests、带 provenance 的回放、板端 observe-only、no-motion intent 和受控动作依次通过。
8. 同步 type、loader/validation、JSON、参数文档、config snapshot、protocol selftest、CMake 和 owner tests；每个公式、状态或阈值只有一个源。

## 7. Control 与 Safety 合同

- yaw target 只由 time-aligned tracking geometry + effective speed 计算；gyro feedback 只在 gate clear 且 motion allow-drive 时运行。
- gate veto、emergency stop、disarmed/no-drive 时不调用 yaw/turn controller，也不更新其 memory。
- wheel mixer 只把 base+turn 映射为左右 target；wheel PID 只从 target+encoder 求 requested PWM；command builder/shaper 只做已声明的限幅与 slew。
- actuator adapter 返回实际 accepted command/apply outcome；PID commit 和 armed observation 以实际 apply 结果为准。
- timer/apply/disable/shutdown 错误必须保持可见；软件命令变为零不证明硬件已经归零。

Safety gate 的 dominant reason 优先级为：

```text
low_voltage
-> perception_stale
-> perception_invalid (projector/health)
-> reference_control_not_ready
-> imu_invalid
-> encoder_invalid
-> initial_reference_hold_not_allowed (仅初次启动边界)
-> none
```

low voltage、perception freshness/health、reference readiness、IMU 和 encoder veto 只属于 gate；感知、reference、yaw 和 debug 层不能各自再实现一份 safety policy。

## 8. 低电压与参数

- `low_voltage_raw_threshold` 属于 power adapter owner；startup 在第一次 sample 前配置它。
- power adapter 的非法/非正 configured threshold 内建 fail-safe 是 400；当前仓库 JSON 值是独立运行输入，不能把 400 写成“当前车辆值”。
- `LS2K_LOW_VOLTAGE_RAW_THRESHOLD` 是显式硬件调试 override，只接受正整数；非法 override 被忽略，不能关闭保护。
- `LowVoltageSample.threshold` 记录实际使用阈值；`low_voltage_sample_interval_ms` 只控制采样周期。

参数加载严格 fail closed；`params.missing/parse/validation/loaded` 的唯一完整操作合同在根 [`README.md`](../../README.md#61-两个启动文件)。本层只规定：`RuntimeParameters{}` 是解析临时对象的字段初值，不是加载失败 fallback 或当前 JSON 的冗余镜像；公式层不得 clamp 非法配置继续运行，也不保留旧 JSON alias。

## 9. Active 文件边界

| 路径 | 当前责任 |
| --- | --- |
| `vision/image/illumination_binary_model.*`、`luma_sampler.*` | 当前 frame 的统一 binary/luma 采样事实 |
| `vision/bev/bev_simple_perception.*` 及 row/boundary/connectivity helpers | sparse road facts 和 ordinary line candidate |
| `vision/elements/cross_exit_element_evidence.*` | Cross evidence |
| `vision/elements/cross_straight_path_planner.*` | Cross straight candidate |
| `vision/elements/zebra_element_evidence.*` | Zebra evidence |
| `vision/elements/zebra_stop_scene.*` | Zebra temporal stop scene |
| `vision/elements/visual_element_pipeline.*` | Cross/Zebra detector 与 Cross planner 编排 |
| `vision/elements/circle_v2/circle_v2_scene.*`、`detail/*` | Circle observers、reducer、composer 和 memory |
| `vision/elements/circle_v2/circle_v2_reference_adapter.*` | Circle reference plan → candidate |
| `vision/ml/ml_observer.*`、`ml_scene.*` | ML facts、maneuver scene/candidate |
| `reference/visual_reference_orchestration.*` | current visual candidate selection |
| `reference/reference_continuity.*` | current/hold/none + hold memory |
| `reference/reference_usability.*` | leading reference usability |
| `reference/reference_tracking_geometry.*` | offset/heading/curvature |
| `reference/reference_control_readiness.*` | reference 控制资格 |
| `runtime/pipelines/steering_frame_pipeline.*` | 单帧阶段顺序、候选政策、唯一 `PerceptionResult` 组装 |
| `safety/control_gate.*` | 唯一 safety veto |
| `control/steering_yaw_controller.*`、mixer/PID/shaper | 分层控制 |
| `transport/steering_media_protocol.*` | 当前 snapshot/config schema serializer |

不存在或未编入 `NEW_SRCS` 的旧 `steering_*.cpp`、full-raster、topology、trusted/memory path 文件不是 active owner。

## 10. Debug、Media 与 Authority

当前 steering snapshot 顶层事实组为：

```text
perception_health
element_evidence
circle_v2
zebra_stop
ml
visual_reference
reference
eligibility
lateral_error
reference_tracking_geometry
reference_time_alignment
reference_control
safety_gate
degraded
yaw_control
actuator
```

config snapshot 从当前 params 复制并由 `steering_media_protocol.cpp` 序列化，包含 speed/yaw/low-voltage、`BEV_PROJECTOR`、`BEV_GEOMETRY`、`BEV_CLASSIFICATION`、`BEV_CONTROL_MODEL`、`BEV_ELEMENT`、ML 和其他当前字段。不要手写另一份带固定数值的 schema；修改参数时以 serializer selftest 证明 snapshot 同步。

- 默认 media 帧必须按控制 snapshot 的 frame id/capture time 对齐；latest-frame 模式只证明最新图像诊断。
- overlay 只画已发布事实；不能在 host 端重建另一个 reference/safety 算法。
- `control.steering_internal` 只用于 tuning evidence，不是运行 authority。
- config 中的 low-voltage 参数不是实际 override 后阈值；后者读取 `LowVoltageSample.threshold`。
- 历史 screenshot、overlay、txt、运行中的 PID 或成功上传不证明当前算法/车辆正确。

## 11. 验证选择

可执行公共 runner、回放、脚本写入风险和证明层级只维护在根 [`README.md`](../../README.md#8-验证每层究竟证明什么)。本专题只负责 owner 选择：元素改动运行对应 Cross、Zebra、CircleV2、ML owner test；控制改动运行 mixer/PID/shaper/motion/stop/apply 对应 runner；涉及真实数据时加入带 input identity 的 aligned replay。随后严格按根文档的通用层级升级，缺少哪层就保留哪层风险。

## 12. 禁止恢复的设计

### 错误事实与路径补偿

禁止：

- 把 unknown、invalid、FOV/image/search boundary 当作 observed boundary/element。
- 为控制需要补点、远端外推、跨 gap 插值或把 hold 伪装成当前视觉点。
- 用 threshold、crop、filter、hold、PID 或 smoothness 掩盖 projector/binary/boundary/evidence 错误。
- 用 `source/mode` 决定 usable、tracking、readiness、gate 或 yaw。

### 全局策略和责任泄漏

禁止恢复旧 topology map、corridor graph、opening/trusted score、trusted path/error blend、camera/fuzzy PID 或跨元素全局策略 FSM。当前 CircleV2、Zebra stop 和 ML maneuver 可以各自维护 scene-owned memory；不得把它们集中为窥探所有元素和控制状态的 scene/policy。

禁止 perception 判断 low voltage，readiness 判断 projector/stale/IMU/encoder，yaw controller 接收 control validity，debug/assistant 重算 speed/reference/safety，或任何非 gate owner 建立第二套 veto。

### 旧参数和伪兼容

active JSON/docs/scripts/media 不得恢复已删除 alias，例如 `emergency_threshold`、`Speed_base`、`see_max`、`PID_TURN_CAMERA`、`PID_TURN_GYRO_CAMERA`、`P_Mode`、`w_target`、`assistant_image_publish_interval_ms`、`assistant_waveform_publish_interval_ms`。需要兼容外部配置时，在明确的迁移工具中 fail/report，不让 production loader 静默解释多个名字。

## 13. Archive 边界

- `new/code/archive/**`、`docs/archive/**`、`new/docs/superseded/**`、`new/verification/archive/**` 不参与 active build、参数、schema 或验收 authority。
- `archive/true_ls2k0300_vendor_drivers_20260713/**` 保存已替代驱动和 manifest，不参与 include/link；当前 LS2K0300 owner 是 `new/code/platform/true_ls2k0300/`。
- active fixture 必须有原始输入、metadata、参数/profile/model/calibration identity 和 hash；历史 overlay/txt/manifest 不能替代输入 provenance。
- 历史版本号、commit 或成功报告只能解释迁移，不能被描述为“当前基线”，除非 release manifest 明确绑定当前源码和资产。

文档与当前编译清单、生产 owner 或可复现 evidence 冲突时，先以它们确定事实，再同步本文；不得让 archive 重新获得 authority。
