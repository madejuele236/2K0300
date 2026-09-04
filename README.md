# 2K0300 智能车源代码：使用、扩展与移植

本仓库保存一套从传感器采集、视觉事实、路径与任务决策、安全门控、闭环控制到硬件执行和证据采集的 C++17 参考实现。当前可运行目标是 LS2K0300 智能车；可复用的核心不是某块板、某条赛道或某个元素阈值，而是各层的输入输出合同、事实所有权、安全边界和分层验证方法。

本文回答四类问题：如何复现当前车辆；如何增删或替换特殊元素；如何完成不同任务；如何换相机、车辆、板卡、OS 或工具链。参数逐项含义、板端命令和 BEV 约束分别只有一个权威来源，本文给出选择规则和闭环路径，不复制会漂移的整表。

> 发布状态：当前工作树可用于继续开发，但尚不是“全新 clone 即可构建”的公开发行包。发布前必须解决 [开源发布门](#开源发布门)：构建依赖有未跟踪路径、没有 LICENSE/NOTICE、运行程序没有内嵌版本身份，而且参数 JSON 与参数说明的当前值合同检查尚未通过。不要用历史构建或板测结果代替当前提交的发布验收。

## 1. 先按目标选择入口

| 目标 | 先读 | 最小改动面 | 最低验收终点 |
| --- | --- | --- | --- |
| 复现当前 2K0300 | 本文第 3、4、8 节；[`new/user/README.md`](new/user/README.md) | 不改源码；提供依赖和配置 | 交叉构建 + 板端 no-motion；需要证明车辆行为时再做受控实车 |
| 只调参数 | [`new/config/default_params.md`](new/config/default_params.md) | 参数 JSON；必要时同步参数合同 | loader 合同 + owner 测试 + 相关回放 + no-motion；运动参数还需受控实车 |
| 更换相机或安装姿态 | 第 9.2 节 | 只改姿态时重标定；换设备时替换 frame-source/backend/device bridge | 新原始帧 + frame/timestamp 合同 + 标定 + 回放 + no-motion + 低速多场景 |
| 不要 Cross/Circle/ML 接管 | 第 10.1 节 | 可先关相应参数 | 参数合同 + ordinary-line 回放 + no-motion |
| 完全不识别任何特殊元素 | 第 10.1 节 | 感知装配；当前不能只靠参数完成 | 元素 absent 合同 + ordinary-line 回放 + no-motion |
| 识别其他特殊元素 | 第 10.2 节 | 新 evidence；按任务再加 candidate/scene/task policy | detector/candidate/scene 单测 + 代表性回放 + observe-only + 受控任务 |
| 完成其他特殊任务 | 第 10.3 节 | 按“观察、换路、停车、速度/动作、无路径任务”选择 owner | 任务状态机合同 + fail-safe + 实际任务证据 |
| 完全不使用图像 | 第 10.4 节 | 替换 perception producer；无 reference 时还要重定义 gate/control | 新传感器合同 + gate + no-motion/台架 + 受控闭环 |
| 更换控制算法 | 第 10.5 节 | yaw、mixer、wheel PID 三层中最小一层 | owner 测试 + gate/停止回归 + 台架 + 低速闭环 |
| 不再使用 2K0300 | 第 11 节 | platform 实现、工厂、构建和部署；上层合同尽量不变 | device contract + 目标构建 + no-motion + 模型数值一致性 + 受控闭环 |

本文命令假定从仓库根目录执行。仓库的自动化环境要求 shell 命令带 `rtk` 前缀；普通终端若未安装该包装器，只去掉每条命令最前面的 `rtk `，不要改动其余参数。

## 2. 什么是当前真相

仓库含多个代际和大量实验/证据；名字更新不代表实现更有效。判断当前行为按以下优先级：

1. `new/user/main.cpp`、`new/user/CMakeLists.txt` 及其编入的 `new/code/**`；它们决定入口和运行字节。
2. 实际启动时的参数文件、硬件 profile、模型/标定资产及其身份。
3. 针对同一源码和资产生成的测试、回放、板端日志与实车证据。
4. active 专题文档。
5. `archive/`、`superseded/`、按日期保存的报告和旧计划；它们只解释历史，不证明当前行为。

### 2.1 目录职责

| 路径 | 当前职责 | 使用规则 |
| --- | --- | --- |
| `new/user/` | 主入口、CMake、构建/部署/采证脚本 | 从这里构建和运行；`debug.sh` 是统一脚本入口 |
| `new/code/port/` | 跨层数据类型和七个硬件抽象接口 | 移植时优先保持；不要用聚合头绕过依赖边界 |
| `new/code/runtime/` | 生命周期、并发、采集、单帧 pipeline、控制循环和服务 | 负责装配，不把错误事实改写成“可用”事实 |
| `new/code/vision/` | 图像/BEV、元素、ML 的事实生产 | detector 只报告能从输入观察到的事实 |
| `new/code/reference/` | 候选仲裁、hold、可用性、跟踪几何、控制就绪 | `source/mode` 只用于解释，不能决定可控性 |
| `new/code/control/` | 运动状态、转向目标、轮目标、轮 PID、命令整形 | 不重新解释感知有效性 |
| `new/code/safety/` | 低电压采样、唯一控制 gate、apply 观察 | veto 归这里；不要在下游伪造通过条件 |
| `new/code/platform/` | 参数/profile、Linux 机制和 LS2K0300 设备实现 | 换平台时替换机制事实，不把设备差异泄漏到算法层 |
| `new/code/transport/`、`observability/` | 序列化已发布事实 | debug 不是 authority，不能反向参与控制 |
| `new/config/` | 仓库默认运行输入和参数合同 | JSON 是输入；loader 是合法性裁判；实际运行可被路径覆盖 |
| `new/verification/tests/` | owner tests、合同探针和保存数据回放 | 先读脚本的输入与写入位置；测试二进制不是源码 |
| `model_training/` | 模型训练、生成器和固定 classifier 资产 | 精度报告不能替代固件 hash/schema/数值 parity |
| `primer_code/`、`old*`、`archive/`、`**/superseded/` | vendor/历史代际/迁移背景 | 不作为当前入口、编译清单或验收 authority |

当前 BEV/reference 的扩展不变量见 [`new/docs/README.md`](new/docs/README.md)，port include/内存所有权见 [`new/code/port/README.md`](new/code/port/README.md)，LS2K0300 设备合同见 [`new/docs/ls2k0300-platform-ownership.md`](new/docs/ls2k0300-platform-ownership.md)。维护流程见 `docs/WORKFLOW.md`；该文件当前存在于工作区但尚未进入 Git HEAD，发布前必须纳入或删除所有对它的依赖。

### 2.2 不要混淆的五组概念

| 概念 | 只表示 | 不表示 |
| --- | --- | --- |
| `BEVPathSample::present` | 这个路径样本事实存在 | 路径可用于控制 |
| `ReferenceUsability::usable` | leading reference 足够连续且几何有限 | safety 已通过 |
| `ReferenceTrackingGeometry::computed` | offset/heading/curvature 已计算且有限 | actuator 可以输出 |
| `ReferenceControlReadiness::ready` | selected reference + tracking geometry 可进入控制 | 电压、时效、IMU、encoder 正常 |
| `ControlGateDecision::veto_active` | 唯一 safety owner 是否禁止控制 | 为什么视觉层产生了某事实 |

`source`、`mode`、置信度、debug overlay 和 telemetry 只解释事实；它们不能替代上述逐层判断。

## 3. 首次构建

### 3.1 当前明确依赖

当前构建不是通用 Linux CMake 工程，存在以下硬绑定：

- Linux/WSL、Bash、CMake、GNU Make、Python 3、GNU 基础工具、SSH 和支持 legacy 模式的 `scp -O`。
- LoongArch GNU 8.3 交叉工具链固定在 `/opt/ls_2k0300_env/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6`；`new/user/cross.cmake` 硬编码编译器。
- 交叉 OpenCV 4.10 固定在 `/opt/ls_2k0300_env/opencv_4_10_build`；CMake 查找路径和目标 ELF RPATH 都引用它。
- `true_LS2K0300_Library/.../zf_components/` 下的 TFLM/NCNN imported archives。
- Python 生成器和所选 ML backend 的固定输入资产；`LS2K_ML_CLASSIFIER` 只接受 `V9` 或 `TFLITE_INT8`。

这些固定路径是当前实现事实，不是推荐的可移植设计。公开发行应把它们变成 CMake cache/toolchain 参数，并提供合法获取方式与 SHA-256；在此之前，新机器缺少任一项就应明确失败，不要复制空目录或用不匹配库“先链接起来”。

### 3.2 只构建，不接触板端

`new/out` 是专用生成目录；`debug.sh build` 会清空其中除占位说明外的内容。不要把手工资产放进去。

```bash
rtk mkdir -p new/out
rtk env SKIP_UPLOAD=1 new/user/debug.sh build
rtk file new/out/new
rtk sha256sum new/out/new
```

`SKIP_UPLOAD=1` 不可省略：`build` 默认还会发现板端并上传。上述结果依次只证明构建脚本成功、产物格式是目标 ELF、这份本地字节的身份；不证明板端能启动或车辆能运动。

需要显式选择 backend 时直接配置 CMake：

```bash
rtk cmake -S new/user -B new/out -DLS2K_ML_CLASSIFIER=V9
rtk cmake --build new/out --target new --parallel
```

或：

```bash
rtk cmake -S new/user -B new/out -DLS2K_ML_CLASSIFIER=TFLITE_INT8
rtk cmake --build new/out --target new --parallel
```

正式产物是 `new/out/new`。默认还生成 `ml_yuyv_capture`、所选 backend 的 board probe 等 helper；`LS2K_BUILD_VERIFICATION_HELPERS=ON` 才编译额外板测目标。不要使用路径相似但不属于本次构建的 `new/build/new`。

## 4. 当前 2K0300：部署、启动与停车

板端网络恢复和 SSH 后端细节见 `docx/README.md` 与项目 `board-access` 工作流；IP、热点和已有运行进程每次都要重新确认。脚本自动发现失败会回退到一个固定 IP，所以首次连接必须显式给出已核验地址。

### 4.1 安全的 no-motion 流程

```bash
rtk env BOARD_IP=<已核验IP> LS2K_REMOTE_BACKEND=auto new/user/debug.sh remote status
rtk env BOARD_IP=<已核验IP> new/user/debug.sh build
rtk env BOARD_IP=<已核验IP> new/user/debug.sh remote start normal
rtk env BOARD_IP=<已核验IP> new/user/debug.sh remote status
rtk env BOARD_IP=<已核验IP> new/user/debug.sh remote logs
```

`build` 上传：

- `new/out/new` → `/home/root/new`
- `new/config/default_params.json` → `/home/root/default_params.json`
- `new/config/hardware_profile.json` → `/home/root/hardware_profile.json`

`remote start normal` 导出板端参数/profile 路径，以 `nohup` 启动，日志写 `/tmp/ls2k/new_runtime.log`，PID 写 `/tmp/ls2k/new_runtime.pid`；脚本一秒后仍见进程就报告启动。`remote logs` 是持续 `tail -f`，需 Ctrl-C 退出。进程存在只证明进程存在，不证明 runtime health。

更安全的一键入口：

```bash
rtk env BOARD_IP=<已核验IP> new/user/start_with_upload.sh no-motion
```

只改参数、不重新编译：

```bash
rtk env BOARD_IP=<已核验IP> new/user/start_with_params_upload.sh no-motion
```

参数单独上传会比较两端 SHA-256；完整 build/upload 当前不会统一核验 binary/profile hash。当前程序也没有 `--version` 或内嵌 commit。每轮板测至少外部保存源码 commit/diff、三份本地/板端文件 SHA-256、classifier/标定身份和启动日志。

### 4.2 运动授权

普通入口默认不发车。只有 no-motion 已证明参数、标定、BEV 观测、gate 和 0 PWM 符合预期，且场地安全时，才显式双重授权：

```bash
rtk env BOARD_IP=<已核验IP> CONFIRM_POWERED_START=1 LS2K_AUTO_START=1 new/user/debug.sh remote restart normal
```

需要自动停车和对齐证据时使用：

```bash
rtk env BOARD_IP=<已核验IP> CONFIRM_POWERED_START=1 new/user/debug.sh steering drive --drive-s 10
```

不要用两个终端手工拼 listener 与 restart；运行时可能先于 listener 回连，导致证据缺失。`start_with_upload.sh drive` 会把自动停车时间置 0，不适合无人看守的短程验收。

正常收车：

```bash
rtk env BOARD_IP=<已核验IP> new/user/stop_car.sh controlled
```

应急立即停：

```bash
rtk env BOARD_IP=<已核验IP> new/user/stop_car.sh now
```

`controlled` 先给进程正常停止窗口，再升级 TERM/KILL；`now` 直接走立即停止路径。进程退出仍必须检查 terminal disable/shutdown 结果，不能仅凭 PID 消失断言硬件输出安全。

## 5. 当前运行时架构

### 5.1 从入口到执行器

当前主入口是 `new/user/main.cpp:main()`，由 `new/user/CMakeLists.txt` 的 `NEW_SRCS` 编入 `new/out/new`。运行链为：

```text
profile + runtime params
-> platform bundle + startup
-> camera capture worker -> fixed-slot frame store
-> perception frontend
-> illumination binary model
-> sparse BEV road facts + ordinary line candidate
-> ML observer/scene + Cross/Zebra pipeline + CircleV2 scene
-> explicit candidate arbitration policy
-> reference continuity (hold)
-> usability -> tracking geometry -> control readiness
-> time alignment
-> safety gate + motion supervisor
-> yaw target/gyro feedback -> wheel target mixer -> wheel PID
-> command builder/shaper -> actuator apply observation
-> assistant/media serialization
```

当前有四个主要执行上下文：相机采集线程、主线程的感知/assistant poll、timer 驱动的控制回调、steering-media 线程。`RuntimeState` 用锁保护感知、传感器、运动、控制快照和帧句柄，用原子量维护生命周期计数。帧存储是固定槽位加 lease/generation，不是无界队列；控制消费“比上次更新的最新帧”。移植时不能默认每帧必处理，也不能把媒体线程当实时控制线程。

candidate arbitration 是显式政策而非全局置信度排序；精确顺序只维护在 [`new/docs/README.md`](new/docs/README.md#1-当前数据流)。新增元素必须在那里定义它与现有 candidate/task 同时出现时的所有权和 memory 复位语义。

### 5.2 Owner 与替换边界

| Owner | 输入 | 唯一输出/副作用 | 不属于它 |
| --- | --- | --- | --- |
| `CameraCaptureWorker` / `CameraFrameStore` | platform frame | 带 frame/time metadata 的有界帧 | 场景解释 |
| `SteeringFramePipeline` | `CameraCapture`、params、motion history | 唯一组装的 `PerceptionResult` | actuator apply |
| sparse BEV perception | pixels、binary model、projector/geometry | row facts、road facts、ordinary candidate | 元素任务和安全策略 |
| element detector | 明确的 rows/ROI/image facts | element evidence | reference、速度或 PWM |
| element candidate builder | grounded evidence | `VisualReferenceCandidate` | hold、gate、执行器 |
| element scene/task reducer | 当前 evidence、owner memory、参数 | 场景状态或明确 task intent | 改写原始 evidence |
| visual orchestration | line/element candidates | current visual reference | hold、safety、yaw |
| continuity | current reference、hold memory | selected current/hold/none | 判断 safety |
| usability/tracking/readiness | selected reference | 分层控制资格事实 | 电压、IMU、encoder veto |
| `EvaluateControlGate` | freshness/health/readiness、power、IMU、encoder | 单一 dominant veto | 生成替代路径 |
| `MotionSupervisor` | intent、gate、phase、停止参数 | phase、allow/stop/reset、effective speed | steering geometry |
| yaw controller | tracking geometry、speed、gyro | turn | 左右轮混合和 apply |
| wheel mixer / PID | base+turn / target+encoder | 左右目标 / requested PWM | safety policy |
| builder/shaper/adapter | PID 输出、边界、previous command | bounded command / accepted apply result | 掩盖 apply 失败 |
| assistant/media | 已发布 snapshot/frame | 命令或序列化证据 | 重新推导运行事实 |

### 5.3 启动、门控与停止状态

启动依次加载 profile、验证 persistence、加载 params、创建 platform、采样低电压、初始化 camera/IMU/encoder/actuator、启动 control/capture/services。profile、params、startup、control loop 或 capture worker 失败时进程返回失败；允许 degraded startup 只用于诊断，不等价于可驾驶。

运动状态：

```text
DISARMED -> START_REQUESTED -> SPINUP -> RUNNING -> STOPPING -> DISARMED
                       \-> FAIL_SAFE_LATCHED <-/
```

drive phase 发生 gate failure 会锁存 `FAIL_SAFE_LATCHED`；只有 gate 清除、满足 rearm hold 且收到显式 reset 才回到 `DISARMED`。停止完成同时要求停止时间、encoder quiet 和整形后命令为零。

gate 依次处理 power、perception freshness/health、reference readiness、IMU、encoder 和初次启动 hold；精确 dominant-reason 顺序只维护在 [`new/docs/README.md`](new/docs/README.md#7-control-与-safety-合同)。初次启动不接受 held/degraded reference；运行中 hold 仍必须经过 usability、tracking、readiness 和 gate。timer 失败会锁存 fail-safe、尝试 disable 并请求退出。apply/disable 未确认时保留失败事实，不能把命令对象置零当作硬件已经归零。

## 6. 配置、模型与标定

### 6.1 两个启动文件

默认路径可由环境变量替换：

- `LS2K_PARAMS_PATH`，默认 `new/config/default_params.json`
- `LS2K_PROFILE_PATH`，默认 `new/config/hardware_profile.json`

参数必填键为 `RUNNING_SPEED_TARGET`、`YAW_RATE_PID.D`、左右 `WHEEL_PID.{P,I,D,INTEGRAL_LIMIT}` 和 `assistant_tcp.{host,port}`；其他键只有在缺失时保留对应 C++ 字段默认，键一旦出现却格式错误、非有限、越界或违反组合算术约束，整体加载失败。

失败语义：

| 诊断 | 条件 | 结果 |
| --- | --- | --- |
| `params.missing` | 文件不可读 | `out` 不变，拒绝启动 |
| `params.parse` | 非严格 JSON 或根不是 object | `out` 不变，拒绝启动 |
| `params.validation` | 必填/可选字段或跨字段约束非法 | `out` 不变，拒绝启动 |
| `params.loaded` | 全部解析和验证成功 | 一次性提交 parsed 参数 |

硬件 profile 必须包含 `camera`、`imu`、`encoder`、`actuator`、`timer`、`persistence`、`display` 七个 block；每个 block 当前形状是 `{"mode":"...","hook":"..."}`。profile 的 `adaptation-hook` 不是移植完成：正常启动仍要求关键设备 direct-match；degraded 模式只允许诊断。

`RuntimeParameters{}` 是字段缺省来源，不是当前车辆参数的副本；JSON 是仓库默认输入，也不等于某次实际运行。不要同时维护两份“相同默认值”。当前 `default_params.md` 与活动 JSON 的值检查存在 13 处失败，因此读当前值应以实际 JSON 为输入、以 production loader 为合法性裁判，发布前必须修复并通过：

```bash
rtk bash new/verification/tests/run_default_params_documentation_contract_test.sh
```

不要在根 README 再复制参数值；逐项语义、单位、范围和联动只维护在 [`new/config/default_params.md`](new/config/default_params.md)。

### 6.2 相机与 BEV

`BEV_PROJECTOR` 是图像到车辆米制坐标的几何根。任何相机型号、分辨率、裁剪、镜头、安装高度或姿态改变，都先重建 raw→BEV 事实，再检查 boundary/reference；不能用 PID、crop、hold 或阈值补偿错误投影。

静止直道、live viewer 已有有效帧时：

```bash
rtk python3 new/user/calibrate_bev_projector_from_live.py
```

该命令只生成结果 JSON 和 overlay。人工确认质量后才显式写回：

```bash
rtk python3 new/user/calibrate_bev_projector_from_live.py --write-params
```

工具有 warning 时默认拒绝写回；不要把 `--force` 写进常规流程。写回后依次跑 loader、BEV owner tests、代表性回放、板端 no-motion 的 raw/BEV/reference/tracking 检查，再做低速直道、偏移和弯道。

### 6.3 ML backend 与资产

当前 classifier 输入是有效 `32x32 gray8` ROI，输出父类 `supplies/vehicle/weapon`；分类、类映射和 maneuver 是不同 owner。`ML.ENABLED=1` 且 `ML.MANEUVER.ENABLED=0` 只观察，不应改变路径或速度。

- V9 由固定 bundle 的 manifest、packed template codes 和 parent table 生成固件资产。
- TFLite identity 由固定 int8 model 与 prototype NPZ 生成；CMake 固定检查两者 hash，runtime artifact identity 覆盖 model bytes 与 NPZ bytes。

替换模型必须同时证明：输入输出 shape/quantization/op contract、生成器 hash、保存数据精度/混淆、host scorer parity、同一 ROI 的板端数值 parity、observe-only 时序和资源预算。训练 accuracy 或模型文件能被打开都不足以证明固件实现正确。详细合同见 [`new/code/vision/ml/README.md`](new/code/vision/ml/README.md)。

## 7. 如何修改而不破坏因果链

先定位会产生或消费目标事实的当前 owner，再按 [`new/docs/README.md` 的扩展合同](new/docs/README.md#6-新元素或新任务的扩展规则)定义输入、输出、状态和同步面。诊断沿原始条件追到第一处错误事实：修复 producer 后症状在同等条件消失才支持根因结论；否则只记录候选原因和未排除项。安全、调试和参数的不可跨越边界也只在该专题维护，根文档不复制第二套规则。

## 8. 验证：每层究竟证明什么

| 层级 | 最小证据 | 能证明 | 不能证明 |
| --- | --- | --- | --- |
| 静态/schema | doc-current、production loader、asset hash/schema | 文件和语义合同自洽 | 算法适合真实输入 |
| owner 单元/合同 | 对应 `run_*_test.sh` | 该 owner 对构造输入的行为 | 当前传感器、调度或整车 |
| 保存数据回放 | raw + metadata + params/profile/model/calibration identity + report/hash | 同输入/同字节下的算法行为 | 新平台采集质量、实时性和闭环 |
| 交叉构建 | `SKIP_UPLOAD=1` 正式构建 | 当前工具链和编译清单能链接 | 上传、目标 ABI、启动 |
| 板端 no-motion | 两端身份、启动日志、0 PWM、live capture | 目标设备加载、初始化、真实静态输入和协议 | 动态控制、赛道完成 |
| 台架/受控短程 | 自动停止、同步 telemetry/raw/config、外部观察 | 动态 gate、转向、轮速、apply 响应 | 全任务和长时稳定性 |
| 完整任务 | 多次正常/失败场景、版本与证据可追溯 | 此版本在此硬件和条件完成目标 | 另一平台、环境或任务 |

### 8.1 按改动选择最小回归

参数：

```bash
rtk bash new/verification/tests/run_param_store_load_runtime_parameters_test.sh
rtk bash new/verification/tests/run_default_params_documentation_contract_test.sh
```

普通 BEV/reference/control 事实链：

```bash
rtk bash new/verification/tests/run_bev_simple_perception_test.sh
rtk bash new/verification/tests/run_visual_reference_orchestration_test.sh
rtk bash new/verification/tests/run_reference_usability_lateral_error_test.sh
rtk bash new/verification/tests/run_reference_tracking_geometry_test.sh
rtk bash new/verification/tests/run_assistant_telemetry_selftest.sh
rtk bash new/verification/tests/run_steering_media_selftest.sh
rtk bash new/verification/tests/run_perf_counter_test.sh
```

低电压/启动：

```bash
rtk bash new/verification/tests/run_power_adapter_threshold_test.sh
rtk bash new/verification/tests/run_startup_low_voltage_order_test.sh
```

保存数据回放按改动选择：

```bash
rtk bash new/verification/tests/run_bev_connectivity_aligned_replay.sh
rtk bash new/verification/tests/run_circle_v2_aligned_replay.sh
rtk bash new/verification/tests/run_cross_straight_aligned_replay.sh
rtk bash new/verification/tests/run_current_pipeline_path_replay.sh
```

运行前读脚本：部分回放写固定 evidence 目录或在 test 目录生成二进制；current-pipeline replay 还依赖 `new/out/generated/v9`。不要把没有 input hash/provenance 的旧报告当发布基线。

构建：

```bash
rtk env SKIP_UPLOAD=1 new/user/build.sh
rtk git diff --check
```

主机测试与构建通过后，按风险升级到 no-motion、台架/受控短程和完整任务。缺少哪一层，就明确保留哪一层风险。

## 9. 常见硬件变化

### 9.1 车体、轮距、编码器或执行器

机械变化先使物理参数和极性合同失效，不先调 PID：

1. 测量/标定 `WHEEL_TRACK_M`、`ENCODER_TICKS_TO_METER`、左右 encoder 正方向、左右 motor/ESC 正方向和 safe-zero。
2. 用 device/mixer/PID/shaper tests 验证 owner 语义。
3. no-motion 确认 encoder 静止、gate 和命令为零。
4. 台架或悬空轮验证方向、饱和、slew 和 disable。
5. 低速直行，再做固定曲率，再恢复完整任务。

LS2K0300 当前逻辑左 motor 绑定物理通道 2、右 motor 绑定通道 1；右 encoder 取反是独立设备合同。移植不能把旧极性照抄成“通用车辆定义”。

### 9.2 更换相机或安装姿态

只改变同一相机的安装高度/角度、镜头或裁剪时，采集机制可以不变，但原有 `BEV_PROJECTOR`、metric ROI 和基于图像位置的标定全部失效；按 6.2 节从新静态 raw frame 重做标定，再做多场景回放和 no-motion。

更换相机设备、驱动、像素格式或采集 API 时，真实采集 owner 不是 legacy `ICameraAdapter::Capture()`：

```text
CameraCaptureWorker
-> MakeStartedCameraFrameSource(params)
-> MakeSourceByName(CAMERA_SOURCE.BACKEND)
-> ICameraFrameSource implementation
-> platform device bridge / driver API
-> CameraFrameStore
```

当前 factory 只接受 `v4l2_yuyv`，device bridge 是 `platform/true_ls2k0300/camera_bridge.cpp`；`CameraAdapter` 只在 startup 校验 profile 和编译期 frame-storage 几何，并明确把 runtime capture 委托给 worker。移植步骤：

1. 在 `port/camera_frame_source.hpp` 的 `ICameraFrameSource` 合同下实现新 backend，并在 `platform/camera_frame_source.cpp:MakeSourceByName` 注册；若底层 ABI 变化，另建对应 device bridge，不能让 worker 识别设备细节。
2. 同步 `CAMERA_SOURCE.{BACKEND,DEVICE,WIDTH,HEIGHT,FPS,BUFFER_COUNT,POLL_TIMEOUT_MS,DRAIN_READY_BUFFERS}` 的 type、loader、JSON、参数文档和 validation；宽高不得超过 `camera_frame_types.hpp` 的编译期 owned storage。
3. 每帧发布真实 format、data lifetime、width/height/stride 和 metadata。当前 downstream 接受的 raw 合同是 YUYV；新格式要么在 source owner 转成已声明格式，要么显式扩展 `CameraFrameFormat` 和全部消费者，不能只改枚举标签。
4. 保证 source 内 `frame_id` 单调；`capture_time_ms` 来自可信硬件/驱动时间，无法使用时明确回退 dequeue time；保留 dequeue time、sequence、timestamp-valid 和 capture timing 诊断。buffer view 只在 consumer 调用期有效，worker 必须在设备 requeue/下一次 capture 前复制到 owned frame slot。
5. 用 source/device contract tests 覆盖 start/stop/restart、协商格式不符、timeout、坏 stride/buffer、时间戳和资源释放；再采集带 metadata 的原始帧，重做 BEV/ROI 标定，执行回放、目标设备 no-motion 和低速多场景。

### 9.3 照明、赛道或标志外观

先采集带 provenance 的原始帧，检查 illumination binary model、投影、row facts 和元素 evidence；只有第一处视觉事实正确但任务边界不合适时才调对应阈值。每次只改变一个 owner 的参数，保存变更前后同帧回放。不要用 reference hold 或控制平滑隐藏 boundary/evidence 错误。

## 10. 改任务、改元素、取消视觉

### 10.1 只保留普通巡线

当前可关闭：

- `BEV_ELEMENT.CROSS_TAKEOVER_ENABLED=0`
- `BEV_ELEMENT.CIRCLE_V2_ENABLED=0`
- `ML.MANEUVER.ENABLED=0`；连 observer 也不需要时设 `ML.ENABLED=0`

这只关闭相应接管/场景，不等于完全删除元素识别。Zebra 当前没有 enabled 参数：每帧仍检测，并可经 Zebra scene 产生 controlled-stop intent。若目标是“任何特殊元素都不识别”，应在 `SteeringFramePipeline::ProcessFrame()` 的元素装配处移除/替换 producer，并同步 candidate 列表、scene memory、task intent、参数、telemetry、CMake 和 tests；不要在 control loop 忽略 Zebra stop 来补偿仍在运行的上游 producer。

保留 ordinary sparse BEV、line candidate、orchestration、continuity、usability、tracking、readiness、gate 和控制链。验收至少包含：特殊输入不会产生元素/任务 intent、普通直/弯/丢边回放不退化、首次启动不靠 hold、no-motion gate 正确。

### 10.2 新增其他特殊元素

先决定新元素产生哪种事实，避免照抄一个不匹配的现有模块：

| 需要 | 参考模式 | 新 owner |
| --- | --- | --- |
| 只识别并记录 | ML observer | evidence + telemetry；不建 candidate/task intent |
| 元素改变局部路径 | Cross/Circle | evidence detector + candidate builder；必要时独立 temporal scene |
| 元素触发停车 | Zebra | evidence + stop scene + 明确 stop intent |
| 元素改变速度/动作 | ML maneuver | classifier/evidence 与 mapping/policy 分离 |
| 同一元素多阶段行为 | CircleV2 | observer facts + reducer state + composer output；每个状态可解释 |

通用的 producer/input/detector/candidate/scene/policy、合同同步和分阶段启用规则只维护在 [`new/docs/README.md`](new/docs/README.md#6-新元素或新任务的扩展规则)。本场景的实际装配点是 `SteeringFramePipeline::ProcessFrame()`、元素自己的类型/owner、`NEW_SRCS`、候选政策和 serializer；先根据上表选模式，再按专题合同接入。验收终点必须包含该元素的 absent/invalid/阈值边界、代表帧、与现有 Zebra/ML/Cross/Circle/ordinary 的冲突和退出、observe-only、no-motion intent；只有产生物理动作时才升级到单元素受控动作和完整任务。

### 10.3 完成其他特殊任务

不要把“任务”都塞进元素 detector。按任务输出选择架构：

#### A. 新任务仍是路径跟踪

新视觉/非视觉 producer 输出 grounded candidate，复用 orchestration → continuity → tracking → gate → yaw/wheel/actuator。例：绕障、分叉选路、泊入引导。新增的任务政策只决定候选或其优先级，不直接计算 PWM。

#### B. 新任务需要停车、等待、重启或分阶段动作

仿照 Zebra 的“evidence → temporal scene → explicit intent”，把任务 phase 放在独立 supervisor/policy。明确每个 phase 的进入证据、最大持续时间、完成条件、中止条件、gate veto 后状态、人工 reset 和终端安全输出。不要用睡眠或 debug 命令充当 runtime 状态机。

#### C. 新任务只改变速度或动作模式

把识别/分类与 class→behavior mapping 分开；mapping 输出受限的 task intent，MotionSupervisor 决定何时允许。新速度仍要满足 wheel target 最坏算术、shaper、gate 和 stop。先 observe-only 发布 mapping，确认混淆和时序后再授权动作。

#### D. 新任务根本没有 reference/path 概念

这不是一个元素插件。定义新的 task-state/control-input 类型和 owner，同时重定义 control gate 中“控制所需事实”。删除不再适用的 projector/reference 条件，并为新传感器 freshness/health/readiness 建立真实信号；不得把 `projector_ok=true`、虚构 reference 或 `ready=true` 填入旧 `PerceptionResult` 以绕过 gate。保留仍成立的 power、IMU/encoder、motion lifecycle、command shaping、apply observation 和 shutdown 合同。

### 10.4 完全不使用图像

当前控制 gate 硬依赖已发布且 fresh 的 perception、`projector_ok` 和 `reference_control.ready`，所以关闭 ML/Circle 或不启动相机都不会得到可驾驶系统。

- 新传感器仍输出路径：替换 `CameraCaptureWorker + PerceptionFrontend/SteeringFramePipeline`，建立新 producer，输出等价的 timestamp/freshness/health、selected reference、tracking geometry 和 readiness；保留 reference/control/safety 或选择在更靠前的层复用。
- 新传感器直接输出 tracking geometry：可把边界提升到 tracking facts，但必须给出 computed/finite/age/quality 的 owner 合同，并重做 replay/同步测试。
- 任务无路径：按 10.3D 重定义 gate 和 control input。

先用记录的传感器数据做确定性回放，再做目标设备 no-motion/台架。不要以“逻辑上会发布”代替真实 timestamp、并发和 stale 行为验证。

### 10.5 替换控制策略

选择最小一层：

- reference geometry → turn：替换 `SteeringYawController`，保留 gate、motion、turn clamp、mixer、PID、shaper。
- base+turn → 左右轮目标：替换 `WheelTargetMixer`，固定正负 turn 和左右轮语义。
- wheel target+encoder → PWM：替换 `WheelPid`，保留 mixer 和 actuator boundary。
- 直接输出 actuator：这是跨越 yaw/mixer/PID 的大替换；仍应保留 gate、motion lifecycle、command shaping、实际 apply observation 和 terminal disable，除非新需求明确逐一替代其合同。

控制测试必须覆盖零值、正负方向、饱和、非有限输入、状态 commit 时机、gate veto 不更新 memory、stop/fail-safe、apply 失败和恢复；随后台架、低速直行、固定曲率、完整任务逐级验收。

## 11. 换平台：不再使用 2K0300

### 11.1 应保持的上层接口

平台边界由 bundle 接口与真实采集接口共同组成；`new/code/port/platform_adapter.hpp` 定义 bundle 中的七个接口：

- `ICameraAdapter`
- `IImuAdapter`
- `IEncoderAdapter`
- `IActuatorAdapter`
- `ITimerAdapter`
- `IPowerMonitorAdapter`
- `IParamStore`
- 聚合它们的 `PlatformBundle`

相机的 runtime capture 不走 `ICameraAdapter::Capture()`；`new/code/port/camera_frame_source.hpp` 另行定义 `ICameraFrameSource`，由 frame-source factory 创建并被 `CameraCaptureWorker` 独占消费。完全换平台必须同时处理这一接口、factory 和 device bridge，不能把七个 bundle 接口称为完整平台面。

新平台实现必须返回正确事实或明确失败；不能让 runtime 根据错误 sample 猜设备状态。尤其要定义：时间单位/单调性、sample 整体有效性、frame buffer 生命周期、encoder 计数和极性、actuator safe-zero/accepted command、timer callback/停止语义、低电压量纲、参数持久化和故障诊断。

### 11.2 当前需替换的绑定点

当前启动是两阶段 bootstrap，而不是先有 `PlatformBundle` 再读取全部配置：

```text
main::LoadProfileAndParams
-> pre-bundle MakeParamStore()
-> load hardware profile
-> require persistence direct-match json-file-store
-> load runtime params
-> CreatePlatformBundle(profile)
-> replace bundle.params with the already-loaded store
```

因此新平台若没有当前 Linux JSON 文件/路径语义，必须先把“选择 pre-bundle store → 加载 profile/params → 选择 platform factory”设计成独立 bootstrap；只替换 `CreatePlatformBundle()` 会在创建新平台前仍依赖旧存储机制。完成 bootstrap 后按以下顺序移植：

1. 新建独立 `new/code/platform/<platform>/`，实现七个 bundle 接口、`ICameraFrameSource` backend/device bridge 和必要 OS 机制；在 factory 注册 backend。pre-bundle 与 bundle 内 `IParamStore` 必须由同一明确工厂/所有权规则产生，避免两套 persistence 语义。相机的 frame/metadata/lifetime 细则按 9.2 节执行。
2. 把 bootstrap/store 选择和 `CreatePlatformBundle()` 从无条件 LS2K0300 构造改为显式 platform factory/构建选择；每次构建只链接一个真实实现。
3. 替换 CMake toolchain、`-march/-mtune`、OpenCV 查找/RPATH、vendor/model 资产路径和部署方式。
4. 替换设备 ABI：当前 LS2K0300 使用 V4L2 YUYV/MMAP、`/dev/zf_*` encoder/PWM/GPIO、IIO ADC/IMU、LoongArch `rdtime.d` 和特定左右通道/符号。
5. 为新实现写 bootstrap/profile/params 和 device contract tests；在目标设备逐项证明 load/init/read/apply/disable/restart/failure，而不是只编译 mock。
6. 若相机或 inference runtime 变化，重做 frame contract、BEV/ROI 标定、模型 op/schema 和同输入数值 parity。
7. 参数/profile 保持 fail-closed；新 profile mode 必须对应真实 factory 能力，不能靠 `adaptation-hook` 文本宣称完成。
8. 部署后比较 binary/config/model/calibration 身份；执行 no-motion，再按 power→sensor→actuator→闭环→任务逐层释放。

### 11.3 平台验收顺序

```text
host/interface tests
-> target cross/native build
-> board device helpers
-> no-motion init + real sensor timestamps + 0 output
-> model same-input parity and performance budget
-> actuator/encoder/IMU bench with emergency stop
-> low-speed straight
-> fixed-curvature tracking
-> special task in isolation
-> repeated complete task + fault cases
```

任何跨平台复用结论只覆盖已跑到的层。原平台的 PWM、轮距、相机、模型性能或赛道结果不自动迁移。

## 12. 观测、证据与问题定位

Steering Media 默认按控制快照的 `frame_id + capture_time_ms` 获取对应相机帧；显式 latest-frame 模式只适合诊断最新画面，不适合证明“这张图导致了这个控制”。网页 viewer 只读，不发送控制命令，也不应重建 runtime reference。

定位顺序：

```text
raw frame / sensor sample
-> capture metadata and freshness
-> projection/binary/row facts
-> element evidence
-> candidate + arbitration
-> selected/held reference
-> usability/tracking/readiness
-> gate dominant reason
-> motion phase/effective target
-> yaw/mixer/PID/shaper
-> accepted actuator apply
-> physical response
```

在第一处错误之前的事实正确、该处 producer 输入足够区分问题、修复后同条件症状消失，才可把它称为根因。否则只报告候选原因和未排除项。

证据 bundle 至少保存：源码 commit 与 dirty diff、构建命令/工具链、binary hash、params/profile bytes 与 hash、model/backend/artifact、标定身份、raw metadata、单调时间、运行日志、对齐 telemetry、外部实车观察和结论边界。历史 overlay、截图、成功命令或运行中的 PID 只证明它们直接观察到的事实。

## 13. 开源发布门

发布一个 tag/压缩包前逐项关闭；未关闭项写进 release notes，不用 README 承诺代替：

- [ ] 选择并加入仓库 LICENSE；审计 vendor、OpenCV、TFLM/NCNN、模型、数据、图片和历史资料的许可证/再分发条件，生成 NOTICE/第三方清单。
- [ ] 确认 `git ls-tree -r HEAD` 含构建所需源码、`docs/WORKFLOW.md`、vendor 获取机制、V9/TFLite 生成输入或带 hash 的下载/bootstrap；当前本机 `.git/info/exclude` 的 `*` 不会随 clone，不能充当发布清单。
- [ ] 把 toolchain、OpenCV、RPATH、vendor/model 路径参数化；记录支持版本和获取方式，提供干净环境构建检查。
- [ ] 让一个全新 clone 在没有开发者本机目录和历史产物时完成 host checks 与正式目标构建。
- [ ] 修复并通过参数 JSON/文档 current-values 合同；区分 C++ 字段缺省、仓库 JSON 和 release params。
- [ ] 提供小型、许可清楚、带 manifest/hash/params/model/calibration 的 canonical fixtures；不要依赖日期目录里的私人历史 evidence。
- [ ] 为 runtime 嵌入或伴随保存 commit/build manifest；部署后统一核验 binary、params、profile、model/calibration identity。
- [ ] 删除或参数化自动发现失败后的固定板端 IP；首次 host key/设备身份必须可核验。
- [ ] 在 release CI 跑 schema、owner tests、canonical replay、两种所支持 backend 和交叉构建；记录哪些测试会写生成物。
- [ ] 在声明支持的真实板卡上跑 no-motion；若 release 声称可驾驶或完成任务，再附受控短程与完整任务的当前版本证据。
- [ ] 检查仓库不含密钥、私人 IP/用户名、绝对本机路径、无许可数据、临时二进制、虚拟环境或不应发布的历史证据。
- [ ] 从零按本文逐条操作一次；删除只能复述、不能改变任何选择/操作/判定的句子，补上任何必须依赖口头传承才能完成的步骤。

## 14. 文档单一事实源

| 内容 | 唯一详表/合同 |
| --- | --- |
| 开源使用、选择、扩展、迁移 | 本文 |
| 构建/板端/steering 命令参数 | [`new/user/README.md`](new/user/README.md) |
| runtime 参数键、单位、范围、联动 | [`new/config/default_params.md`](new/config/default_params.md) |
| BEV/reference/element 分层不变量 | [`new/docs/README.md`](new/docs/README.md) |
| port 类型、include 和 memory ownership | [`new/code/port/README.md`](new/code/port/README.md) |
| LS2K0300 driver/设备合同 | [`new/docs/ls2k0300-platform-ownership.md`](new/docs/ls2k0300-platform-ownership.md) |
| ML runtime/backend/asset 合同 | [`new/code/vision/ml/README.md`](new/code/vision/ml/README.md) |
| 当前维护工作流 | `docs/WORKFLOW.md`（发布前纳入 Git） |

文档与源码冲突时，以当前编译清单、生产 owner、实际输入字节和可复现证据为准；随后修正文档。任何 archive/superseded 内容都不得重新获得运行时 authority。
