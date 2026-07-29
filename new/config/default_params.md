# `default_params.json` 调参指南

本文只描述当前 active runtime 仍会读取和发布的参数。旧复杂寻线、元素识别、拓扑、roadblock 占位和历史 PID 命名已归档，不再作为运行时、协议或调参依据。

当前闭环固定为：

```text
frame -> 20x15 illumination-compensated residual binary model -> sparse BEV boundary facts -> visual reference facts -> reference usability -> tracking geometry
-> reference-control readiness -> safety gate -> yaw-control terms -> actuator
```

`new/config/default_params.json` 是人工编辑的运行默认合同。`RuntimeParameters` 内建值只为可选字段缺省提供逐字段初值；缺文件、JSON 解析失败或任何字段/组合校验失败都会拒绝启动，绝不会整包 fallback 到内建参数。必须通过 `run_param_store_load_runtime_parameters_test.sh` 校验活动 JSON 的 schema、解析与参数约束。

## 1. 调参前提

每次只调整一个明确参数族，并先写清楚要验证的假设。不要把曝光、BEV 标定、reference usability、lateral-error、yaw PID、轮速 PID 和运动状态机混在一次修改里。

推荐现场闭环：

```bash
cd new/user
rtk ./debug.sh assistant on 192.168.137.1 43011 43012
rtk ./start_with_upload.sh no-motion
rtk env LS2K_HOST_CAPTURE_BACKEND=windows ./debug.sh steering host-capture --listen-host 0.0.0.0 --listen-port 43011 --media-listen-port 43012 --duration-s 20
```

Windows 热点链路优先使用当前高端口配置。`debug.sh` 会在 `BOARD_IP` 未显式设置时自动发现热点板端，并在 `192.168.137.x` 下自动使用 Windows OpenSSH/SCP；`host-capture` 的 Windows 后端会先写本地临时目录，结束后再复制回 WSL evidence 目录。

证据先看这些分组：

- `perception_health.{projector_ok,reason}`：投影和感知健康。
- `reference.{mode,source}`：selected reference 路径事实来源。
- `perception_tag`、`boundary_row_count`、`boundary_jump_count`、`boundary_span_count`：V9 sparse boundary facts 是否持续发布。
- `eligibility.{usable,leading_usable_samples,leading_min_forward_m,leading_max_forward_m,reason}`：reference facts 是否足够连续。
- `lateral_error.{computed,weighted_lateral_error_m,weighted_sample_count,weight_sum,reason}`：legacy weighted lateral-error 迁移对照事实，不再是 V6 主控输入。
- `reference_tracking_geometry.{computed,lateral_offset_m,heading_error_rad,curvature_m_inv,sample_count,reason}`：V6 reference-control readiness 和 yaw target 的权威几何输入。
- `reference_control.{ready,reason}`：reference + tracking geometry 是否可进入控制。
- `safety_gate.{veto_active,reason}`：唯一安全 gate，独占低电压、感知健康、stale、IMU、encoder 否决。
- `yaw_control.{lateral_term,heading_term,curvature_term,turn_output_target}`：tracking geometry 三项组合后的 turn-output 目标，单位与左右轮速半差一致。
- `actuator.{raw_turn_output,applied_turn_output}`：最终 turn-output。`applied_turn_output` 是控制器请求的 turn-output，最终左右轮速由 mixer 按加速侧/减速侧 scale 生成；非对称 scale 会改变左右目标的平均值。

最小离线回归：

```bash
rtk bash new/verification/tests/run_param_store_load_runtime_parameters_test.sh
rtk bash new/verification/tests/run_power_adapter_threshold_test.sh
rtk bash new/verification/tests/run_startup_low_voltage_order_test.sh
rtk bash new/verification/tests/run_bev_simple_perception_test.sh
rtk bash new/verification/tests/run_visual_reference_orchestration_test.sh
rtk bash new/verification/tests/run_reference_usability_lateral_error_test.sh
rtk bash new/verification/tests/run_assistant_telemetry_selftest.sh
rtk bash new/verification/tests/run_steering_media_selftest.sh
rtk bash new/verification/tests/run_perf_counter_test.sh
```

## 2. 加载语义

<!-- contract:params-load-fail-closed codes=params.missing,params.parse,params.validation out=unchanged startup=refused -->

运行时由 `new/code/platform/param_store.cpp` 读取 `new/config/default_params.json`。

- 缺文件：发布 `params.missing` 并拒绝启动，输出参数保持不变。
- JSON 语法非法或根节点不是 object：发布 `params.parse` 并拒绝启动，输出参数保持不变。
- 必填字段缺失、字段类型/范围错误或可选字段违反生产算术合同：发布 `params.validation` 并拒绝启动，输出参数保持不变。

当前必填键：

- `RUNNING_SPEED_TARGET`
- `YAW_RATE_PID.D`
- `LEFT_WHEEL_PID.{P,I,D,INTEGRAL_LIMIT}`
- `RIGHT_WHEEL_PID.{P,I,D,INTEGRAL_LIMIT}`
- `assistant_tcp.{host,port}`

其余键都是可选覆盖；缺省时使用内建默认值。可选键格式错误或越界仍会触发整体 `params.validation`，拒绝启动而不是回退后继续运行。

## 3. 诊断到参数的顺序

1. 没有 host 连接或数据很少：先看 `assistant_tcp.*`、`assistant_enabled`、`steering_media_*`，再看板端 `assistant.backoff`、`steering_media.backoff`、`steering_media.summary`。
2. 边界事实不对：先看对齐的 raw/binary 图像、`otsu.{valid,threshold,source,stale_frames}`、`BEV_PROJECTOR`、`BEV_GEOMETRY` 和 `boundary_jump_count/boundary_span_count`；当前 runtime 不提供曝光控制参数。
3. 边界事实对但 `eligibility.usable=false`：看 hold 周期 `BEV_CLASSIFICATION.HOLD_LAST_MAX_CYCLES`、`BEV_CONTROL_MODEL.MIN_LEADING_REFERENCE_SAMPLES`、`BEV_GEOMETRY.FORWARD_SAMPLE_*`。
4. reference 对但 `reference_tracking_geometry` 不合理：看 leading reference path、boundary spans/traces、`BEV_CONTROL_MODEL.TRACKING_FIT_MIN_SAMPLES`，并用 `lateral_error` 只做迁移期对照。
5. tracking geometry 合理但转向幅度不对：看 `BEV_CONTROL_MODEL.LATERAL_OFFSET_TO_WHEEL_DELTA_GAIN`、`BEV_CONTROL_MODEL.HEADING_ERROR_TO_WHEEL_DELTA_GAIN`、`BEV_CONTROL_MODEL.CURVATURE_TO_WHEEL_DELTA_GAIN`、`YAW_RATE_PID.*`、`raw_turn_output_limit`、`wheel_turn_accel_delta_scale`、`wheel_turn_decel_delta_scale`。
6. `element_evidence.cross_exit` 与画面不一致：先看连续 boundary absence 和 `element_evidence.cross_exit.{boundary_jump_count,boundary_span_count,boundary_absent_row_count,reason}`。
7. 直行速度或左右轮跟随不对：看 `RUNNING_SPEED_TARGET`、`LEFT_WHEEL_PID.*`、`RIGHT_WHEEL_PID.*`。
8. 起步、停止、fail-safe 恢复节奏不对：看 `motion_*`、`pwm_limit`、`pwm_floor`、反转保护和低电压参数。

## 4. 速度、yaw 和轮速 PID

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `RUNNING_SPEED_TARGET` | `200.0` | motion supervisor / yaw speed scale | 运行轮速目标单位，不是 m/s，合法范围为 `[0, 5000]`。增大后车速更高，yaw target 也会按 speed scale 变化。看 `effective_speed_target`、左右 `*_speed_target`、encoder measured。先用低值确认闭环再上调。 |
| `YAW_RATE_PID.P` | `0.0` | gyro feedback | gyro yaw-rate 对 turn-output 的反馈修正增益。它不承担 reference tracking geometry 前馈/反馈幅度；摆动或 raw turn 频繁反向时先看它，单纯欠转先看 BEV control model 的三项 gain。该值不是任意 `double`：必须能转换为有限生产 `float`，并与 I/D 一起通过联合最坏算术检查。 |
| `YAW_RATE_PID.I` | `0.0` | gyro feedback | gyro 反馈积分。当前默认不用。只有长期同向 gyro 偏差且 P/D 不能解决时小幅增加；积分过大会拖尾。该值必须能转换为有限生产 `float`，并与 P/D 一起通过联合最坏算术检查。 |
| `YAW_RATE_PID.D` | `0.0` | gyro feedback | 抑制 gyro 反馈误差变化。抖动和过冲明显时增加；过大时转向变钝。该值必须能转换为有限生产 `float`，并与 P/I 一起通过联合最坏算术检查。 |
| `LEFT_WHEEL_PID.P` | `84.0` | 左轮速度 PID | 左轮速度误差主增益。左轮跟随慢增大；PWM 抖或超调减小。看 `left_speed_target`、`left_measured_speed`、`left_drive_pwm_command`。 |
| `LEFT_WHEEL_PID.I` | `4.8` | 左轮速度 PID | 左轮长期误差积分。稳态低于目标时增大；起步后拖尾或积累过冲时减小。 |
| `LEFT_WHEEL_PID.D` | `0.75` | 左轮速度 PID | 左轮速度变化阻尼。速度抖动可增大；响应迟钝可减小。 |
| `LEFT_WHEEL_PID.INTEGRAL_LIMIT` | `5000.0` | 左轮速度 PID | 左轮积分上限。积分饱和导致恢复慢时减小；长期负载跟不上且 I 有效时可增大。 |
| `LEFT_WHEEL_PID.MEASUREMENT_FILTER_ALPHA` | `0.4` | 左轮速度测量滤波 | 越大越信当前测量，响应快但噪声多；越小越平滑但滞后。看 measured speed 噪声和 PWM 震荡。 |
| `RIGHT_WHEEL_PID.P` | `96.0` | 右轮速度 PID | 右轮速度误差主增益，方法同左轮。左右参数可独立调节，不要为了对称而强行改成一样。 |
| `RIGHT_WHEEL_PID.I` | `4.4` | 右轮速度 PID | 右轮长期误差积分，方法同左轮。 |
| `RIGHT_WHEEL_PID.D` | `0.2` | 右轮速度 PID | 右轮速度变化阻尼，方法同左轮。 |
| `RIGHT_WHEEL_PID.INTEGRAL_LIMIT` | `5000.0` | 右轮速度 PID | 右轮积分上限，方法同左轮。 |
| `RIGHT_WHEEL_PID.MEASUREMENT_FILTER_ALPHA` | `0.4` | 右轮速度测量滤波 | 右轮测量滤波，方法同左轮。 |

`YAW_RATE_PID.P/I/D` 没有三个彼此独立的固定上限。loader 和控制器复用同一个联合合同：IMU producer 的 raw 与启动 bias 都是 `int16`，最大计数差为 `65535`，乘 `0.0010641 rad/s/count` 后，生产 `gyro_z`/error 的可达幅值约为 `69.75 rad/s`，derivative 输入幅值最多约为其两倍；积分记忆限制为 `1200`，turn target 限制为生产 `float(INT_MAX)`。P/I/D 必须先各自转换为有限 `float`，再保证按控制器真实的 `turn target + P + I + D` 顺序，在上述 producer 可达极值下每一步都保持有限。输出 clamp 位于这些检查之后，不能把 `NaN` 或溢出的中间结果“消毒”为合法输出。

<!-- contract:yaw-path assistant=yaw_control.valid/reason media=steering_snapshot.yaw_control.valid/reason -->

如果 JSON 中的 YAW 配置违反该合同，参数加载会以 `params.validation` fail closed，程序不会带着该配置启动。若程序化状态或运行输入为非有限值、越过 producer/控制器合同，yaw 计算会返回 `valid=false` 和具体 `reason`；control loop 会在整数 `round/cast` 前将 gate 置为 `yaw_control_invalid`，让 MotionSupervisor 进入现有 `FAIL_SAFE_LATCHED`/emergency-stop 路径，并跳过 wheel mixer 与 wheel PID、清零 turn/轮速目标。排查时看 `control.steering_snapshot` 的 `yaw_control.valid/reason`：assistant telemetry 的 JSON 根级路径为 `yaw_control.valid/reason`，steering media 的路径为 `steering_snapshot.yaw_control.valid/reason`。同时检查 `control.yaw.invalid` 与 `control.veto.yaw_control_invalid` 诊断。

左右轮 PID 使用基于最终有刷驱动 PWM 的条件积分 anti-windup。硬限幅、全局步进限制和禁止反转会提供明确的正/负方向 blocking facts；只有候选积分继续把输出推向该受阻方向时，才冻结本周期积分，候选积分能帮助退出受阻方向时仍允许提交。`pwm_floor` 只调整最低非零幅值，整数 rounding 只完成 PWM 量化，两者都不单独设置 blocking direction，因此不会仅因 floor/rounding 冻结积分，也允许积分穿越 floor 和零点。执行器未实际施加命令时冻结积分。遥测可通过 `*_drive_pwm_unconstrained/requested/desired/command`、`*_drive_pwm_step_limited/reverse_suppressed/floor_adjusted`、`*_pid_error/integral/integral_candidate` 与 `*_pid_anti_windup_active/reason` 还原该决策链；若未施加冻结，reason 为 `not_applied`，受明确方向约束冻结时为 `actuator_limit`，未冻结时为 `none`。

## 5. 相机与基础时序

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `CAMERA_SOURCE.BACKEND` | `v4l2_yuyv` | camera capture worker | 主相机源。默认直接走 V4L2 YUYV，避免 supplier MJPG/OpenCV 转换进入 foreground perception path。 |
| `CAMERA_SOURCE.DEVICE` | `/dev/video0` | camera frame source | V4L2 设备路径。换摄像头设备名时只改这里。 |
| `CAMERA_SOURCE.WIDTH` / `HEIGHT` | `320` / `240` | camera frame source | source 输出几何，必须不超过编译期 frame storage。 |
| `CAMERA_SOURCE.FPS` | `60` | camera frame source | 请求帧率；driver 可能协商失败，实际以 camera source health/perf 为准。 |
| `CAMERA_SOURCE.BUFFER_COUNT` | `3` | camera frame source | V4L2 mmap buffer 数。过小容易丢帧，过大可能增加队列滞后。 |
| `CAMERA_SOURCE.POLL_TIMEOUT_MS` | `50` | camera capture worker | capture thread 内等待上限；不阻塞 main/control loop。 |
| `CAMERA_SOURCE.DRAIN_READY_BUFFERS` | `1` | V4L2 capture owner | `0` 时 poll 后只尝试一次成功 DQBUF；`1` 时在 mmap buffer 数量上界内继续非阻塞 DQBUF，直到 `EAGAIN`，旧 buffer 立即回队并保留最新有效 buffer。`drained_buffer_count` 记录本次所有成功 DQBUF 次数。 |
| `control_period_ms` | `5` | control timer | 控制 tick 周期，合法范围 `[1, 1000]ms`。减小会提高 CPU/IO 压力；增大会降低控制响应。看 perf、`control.tick` 和实际电机稳定性。 |
| `perception_stale_ms` | `120` | safety gate | 最新 perception 超过该时间即 stale，合法范围 `[1, 86400000]ms`（最大 24h）。摄像头偶发慢帧可适当增大；过大则会让旧 reference 继续影响控制。看 `safety_gate.reason=perception_stale`。 |
| `control_snapshot_emit_interval_ms` | `100` | debug reporter | 板端 `control.snapshot` 与 `control.steering_snapshot` 输出周期，合法范围 `[1, 86400000]ms`（最大 24h）。只影响日志密度，不改变控制。 |

`REFERENCE_TIME_ALIGNMENT` 是控制侧 reference 时间坐标对齐参数，不属于视觉识别：

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `REFERENCE_TIME_ALIGNMENT.ENABLED` | `0` | control-side reference facts | 开启后控制侧在计算 usability/lateral error/tracking geometry/readiness 前，把 reference 从 capture time 对齐到 control-effective time。默认关闭，保持当前运行行为。 |
| `REFERENCE_TIME_ALIGNMENT.MAX_AGE_MS` | `120` | reference time alignment | reference 最大可对齐年龄。超过说明视觉事实太旧，fail closed。 |
| `REFERENCE_TIME_ALIGNMENT.EFFECTIVE_DELAY_MS` | `0` | control loop orchestration | `now_ms -> control_effective_time_ms` 的估计延迟。未完成板端延迟标定前保持 0。 |
| `REFERENCE_TIME_ALIGNMENT.FUTURE_PREDICTION_MAX_MS` | `80` | vehicle pose delta estimator | 允许从当前控制时刻预测到 control-effective time 的最大未来窗口。 |
| `REFERENCE_TIME_ALIGNMENT.MAX_INTEGRATION_GAP_MS` | `30` | motion history | motion history 允许的最大采样空洞。 |
| `REFERENCE_TIME_ALIGNMENT.MIN_ALIGNED_SAMPLES` | `3` | reference time alignment | 对齐后最少前方样本数。 |
| `REFERENCE_TIME_ALIGNMENT.USE_ENCODER_FORWARD` | `0` | vehicle pose delta estimator | 是否用编码器积分前向位移。默认关闭，直到共享的 `MOTION_ODOMETRY.ENCODER_TICKS_TO_METER` 实测完成。 |
| `REFERENCE_TIME_ALIGNMENT.WHEEL_TRACK_M` | `0.154` | vehicle pose delta estimator | 左右轮中心距，用于 IMU yaw 不可用时的 wheel-yaw fallback。实测外缘总宽 `0.18m`，单轮厚 `0.026m`，所以中心距为 `0.18-0.026=0.154m`；编码器尺度仍未知，因此 fallback 保持关闭。 |
| `REFERENCE_TIME_ALIGNMENT.USE_IMU_YAW` | `1` | vehicle pose delta estimator | 是否优先使用 IMU `gyro_z` 积分 yaw。 |
| `REFERENCE_TIME_ALIGNMENT.USE_WHEEL_YAW_FALLBACK` | `0` | vehicle pose delta estimator | IMU yaw 不可用时是否使用左右编码器差估 yaw。默认关闭，直到轮距和编码器尺度标定完成。 |
| `REFERENCE_TIME_ALIGNMENT.FUTURE_PREDICTION_ENABLED` | `0` | vehicle pose delta estimator | 是否允许预测 `now_ms -> control_effective_time_ms`。默认关闭。 |
| `REFERENCE_TIME_ALIGNMENT.COMMAND_YAW_PREDICTION_ENABLED` | `0` | vehicle pose delta estimator | 是否使用已施加 turn output 预测未来 yaw rate。默认关闭。 |
| `REFERENCE_TIME_ALIGNMENT.TURN_OUTPUT_TO_YAW_RATE_GAIN` | `0.0` | vehicle pose delta estimator | `applied_turn_output -> yaw_rate(rad/s)` 的实测增益，合法范围 `[-100, 100]`。 |
| `REFERENCE_TIME_ALIGNMENT.ACTUATOR_YAW_TAU_MS` | `35.0` | vehicle pose delta estimator | 命令 yaw 预测的一阶执行响应时间常数，合法范围 `[0, 1000]`。 |
| `REFERENCE_TIME_ALIGNMENT.MAX_DELTA_FORWARD_M` | `0.782641706` | reference time alignment | 旧虚构 BEV 前向上限 `0.6` 按前向尺度换算后的单次对齐上限；尚未由运动实测重新标定。 |
| `REFERENCE_TIME_ALIGNMENT.MAX_DELTA_LATERAL_M` | `0.441152591` | reference time alignment | 旧虚构 BEV 横向上限 `0.4` 按横向尺度换算后的单次对齐上限；尚未由运动实测重新标定。 |
| `REFERENCE_TIME_ALIGNMENT.MAX_DELTA_YAW_RAD` | `0.8` | reference time alignment | 单次对齐允许的最大 yaw 积分量，超限 fail closed。 |

组合约束在参数加载阶段提前校验：当 `REFERENCE_TIME_ALIGNMENT.ENABLED=1`
时，`USE_ENCODER_FORWARD=1` 要求 `MOTION_ODOMETRY.ENCODER_TICKS_TO_METER > 0`；
`USE_WHEEL_YAW_FALLBACK=1` 要求该共享比例大于 0 且
`WHEEL_TRACK_M > 0`；`COMMAND_YAW_PREDICTION_ENABLED=1` 要求
`FUTURE_PREDICTION_ENABLED=1` 且 `TURN_OUTPUT_TO_YAW_RATE_GAIN != 0`。
<!-- contract:reference-time-alignment-invalid action=params.validation out=unchanged startup=refused -->

这些组合不满足时，参数加载发布 `params.validation`、保持输出参数不变并拒绝启动；不会回退默认参数，也不会把错误组合留到 estimator 运行时处理。

## 6. 执行器与运动状态机

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `pwm_limit` | `5000` | actuator safety | 左右轮 PWM 绝对限幅。车无力且 PID 未饱和时不要先改它；只有确认输出长期被限幅且硬件允许时上调。 |
| `raw_turn_output_limit` | `20000` | turn output safety | 配置值合法范围为 `[0, INT_MAX]`，正 `INT_MAX` 仍合法，单位与左右轮速半差一致。实际进入 wheel mixer 的幅值上限为 `T = min(raw_turn_output_limit, 9000)`；它是兜底边界，不是常规转向幅度调参旋钮。 |
| `wheel_turn_accel_delta_scale` | `1.0` | wheel target mixer | 差速混合中加速侧 turn delta 缩放系数，必须有限且非负。正 turn 时左轮使用该系数，负 turn 时右轮使用该系数；参数加载还会验证 `T * scale` 和 `base + T * scale` 按生产运算顺序均保持 finite。 |
| `wheel_turn_decel_delta_scale` | `2.0` | wheel target mixer | 差速混合中减速侧 turn delta 缩放系数，必须有限且非负。正 turn 时右轮使用该系数，负 turn 时左轮使用该系数；参数加载还会验证 `T * scale` 保持 finite，减速侧目标低于 0 时仍 clamp 到 0。 |
| `pwm_floor` | `0` | actuator shaping | 非零 PWM 的最小地板。低速克服静摩擦可小幅上调；过高会让轻微控制也变成突跳。 |
| `prohibit_reverse_pwm` | `1` | actuator safety | 禁止输出反向 PWM。负请求先变为期望值 `0`，再由全局驱动 PWM 步进限制缓降。 |
| `drive_pwm_step_limit` | `2000` | actuator safety | 左右有刷驱动 PWM 每控制周期最大变化量；覆盖 SPINUP、RUNNING 和受控 STOPPING，急停立即归零。 |
| `brushless_debug_fixed_pwm_enabled` | `0` | actuator debug | 启用后，正常可驱动周期把左右无刷电调命令固定为 `brushless_debug_fixed_pwm`；关闭后左右无刷电调命令为 `0`。 |
| `brushless_debug_fixed_pwm` | `900` | actuator debug | 无刷电调固定调试 PWM，合法范围 `[0, 1000]`。该值只进入统一 `ActuatorCommand` 的左右无刷字段，不在 adapter/bridge 内隐藏生成。 |
| `motion_unveto_confirm_cycles` | `3` | motion supervisor | safety gate 解除后需要连续干净周期数，必须至少为 `1`，且 `motion_unveto_confirm_cycles * control_period_ms <= 86400000ms`（24h）。误解除风险高时增大；恢复太慢时减小。 |
| `motion_spinup_ms` | `800` | motion supervisor | 起步速度爬升时间，合法范围 `[0, 86400000]ms`；`0` 表示立即完成 spinup。起步打滑或冲击大时增大；起步太慢时减小。 |
| `motion_turn_limit_spinup` | `1.0` | motion supervisor | 起步阶段转向限幅比例，必须 finite 且在 `[0, 1]`。起步时转向过猛减小；起步弯道跟不上增大。 |
| `motion_stop_ms` | `300` | motion supervisor | stop 阶段速度衰减时间，合法范围 `[0, 86400000]ms`；`0` 表示立即完成时间衰减，但退出 STOPPING 仍要求 encoder quiet 且 shaped command 为零。停车太急增大；停车拖尾减小。 |
| `motion_stop_encoder_threshold` | `8` | motion supervisor | 判定停止的 encoder 阈值，合法范围 `[0, 5000]`。车已停但不退出 STOPPING 可增大；未停就退出可减小。 |
| `motion_fault_rearm_hold_ms` | `600` | motion supervisor | fail-safe latch 后允许 rearm 前的保持时间，合法范围 `[0, 86400000]ms`；`0` 表示不附加时间保持，但其他 rearm 条件仍须满足。现场排障保守时增大；恢复流程过慢时减小。 |

上述有限性检查中的 `base` 是可配置的基础轮速目标：`RUNNING_SPEED_TARGET` 的合法范围是 `[0, 5000]`；`ML.ENABLED=1` 时可达的 `ML.MANEUVER.SPEED_TARGET` 合法范围是 `(0, 5000]`。当 `T=0` 时，任意 finite、非负的 accel/decel scale 都合法；当 `T>0` 时，scale 必须通过表中与生产同序的乘法、加法有限性检查。任何越界或产生 non-finite 中间结果的配置都会由 loader fail closed，拒绝启动，不会留给 mixer 或遥测层补救。

调度间隔和状态机时间窗共享单项最大值 `86400000ms`（24h）；只有消费方定义了立即语义的窗口允许 `0`。确认周期不是独立的无界整数，其与 `control_period_ms` 的乘积同样不得超过 24h。
<!-- contract:param-semantic-bounds ports=[1,65535] control-period-ms=[1,1000] runtime-window-max-ms=86400000 motion-turn-spinup=finite[0,1] motion-stop-encoder=[0,5000] low-voltage-threshold=[1,INT_MAX] raw-turn-output=[0,INT_MAX] -->
<!-- contract:motion-confirmation-window cycles-min=1 product-ms=<=86400000 -->

## 7. Low Voltage 与调试传输

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `low_voltage_raw_threshold` | `200` | power adapter / safety gate | ADC raw 低电压阈值，合法范围 `[1, INT_MAX]`；正 `INT_MAX` 仍合法。实际使用值记录在 `LowVoltageSample.threshold`；`LS2K_LOW_VOLTAGE_RAW_THRESHOLD` 环境变量优先。误报低电压时先查 ADC raw，再谨慎下调；超大正数会更保守。 |
| `low_voltage_sample_interval_ms` | `1000` | low-voltage sampler | 运行期低电压采样周期，合法范围 `[1, 86400000]ms`（最大 24h）。默认 1Hz；降低会增加 IO，升高会降低低电压发现速度。 |
| `assistant_enabled` | `1` | assistant TCP | 是否启用 command/ACK/telemetry 链路。连接调试时保持开启；纯离线运行可关闭。 |
| `assistant_tcp.host` | `192.168.137.1` | assistant TCP | 板端主动连接的 host 地址。Windows 热点链路通常是 `192.168.137.1`；错误时板端会 `assistant.backoff Connection refused/timeout`。 |
| `assistant_tcp.port` | `43011` | assistant TCP | host assistant listener 端口，合法范围 `[1, 65535]`。必须和 `debug.sh assistant on/local` / `debug.sh steering host-capture` / `tune_speed.py` 一致。Windows 热点链路优先使用高端口，避免低端口被系统策略拒绝绑定。 |
| `steering_media_enabled` | `1` | steering media TCP | 是否启用图像和 steering snapshot side channel。调视觉/boundary facts 时保持开启；带宽或 CPU 排查时可临时关闭。 |
| `steering_media_port` | `43012` | steering media TCP | host media listener 端口，合法范围 `[1, 65535]`。必须和 `--media-listen-port` 一致。 |
| `steering_media_publish_interval_ms` | `100` | steering media service | 图像发布间隔，合法范围 `[0, 86400000]ms`（最大 24h）；`0` 表示每个 eligible tick 都可发布。`100ms` 理论上约 `10fps`；实际看 host `effective_fps` 和板端 `steering_media.summary.skip_interval/image_sent/image_queued`。弱热点链路优先降位深或降采样，确认队列不堆积后再压低该间隔。 |
| `steering_media_downsample` | `1` | steering media service | 图像 side channel 的发送降采样倍率。`1` 保留 320x240 显示尺寸；热点链路吞吐不足时可临时设为 `2`/`4`，header 仍保留 source 尺寸和 downsample。 |
| `steering_media_gray_bits` | `8` | steering media service | 图像传输灰度位深。支持 `1/2/4/8`。当前 `8` 发送原始 gray8；热点吞吐不足时可用 `2` 的 `gray2_packed` 降至四分之一带宽，或进一步改用 `1`。 |
| `steering_media_publish_latest_frame` | `0` | steering media service | 诊断开关。默认 `0` 时图像帧与 `control.steering_snapshot` 精确强绑定；显式置 `1` 或脚本 `--media-latest-frame` 才会发布最新相机帧并在 header 标出非对齐状态。 |
| `steering_media_publish_disarmed` | `1` | steering media service | 是否允许 DISARMED/no-motion 状态发布图像帧。静态采集、BEV 调参和赛道外取证时保持开启；关闭时 host 只能收到 config，板端 `steering_media.summary.skip_disarmed` 会增长。 |

## 8. BEV Projector 标定

`BEV_PROJECTOR` 定义原图到车辆坐标系的投影。它是 sparse boundary facts 的几何根，错误时后续所有参数都会被误导。

| 参数 | 当前 JSON 值 | 调参方法与证据 |
| --- | --- | --- |
| `BEV_PROJECTOR.VALID` | `1` | 投影是否可用。置 `0` 会让 perception health 失败，只用于 fail-safe 验证。 |
| `BEV_PROJECTOR.PROJECTOR_ID` | `bev_projector_red_marker_metric_20260713T171917Z` | 标定版本名。只改标识，不改变几何；更新标定时同步改。 |
| `BEV_PROJECTOR.PROJECTOR_HASH` | `bev-projector-red-marker-0p12x0p05-20260713T171917Z` | 标定版本 hash/说明。只用于身份和 LUT 重建判断。 |
| `BEV_PROJECTOR.DEBUG_GRID_WIDTH` | `160` | dense debug BEV 图宽度，只影响调试图，不是 runtime sparse/raster authority。 |
| `BEV_PROJECTOR.DEBUG_GRID_HEIGHT` | `128` | dense debug BEV 图高度，只影响调试图。 |
| `BEV_PROJECTOR.SOURCE_ROW_0` / `SOURCE_COL_0` | `219.0` / `55.5` | 近端左标定点在原图中的像素位置。 |
| `BEV_PROJECTOR.SOURCE_ROW_1` / `SOURCE_COL_1` | `219.0` / `307.5` | 近端右标定点在原图中的像素位置。 |
| `BEV_PROJECTOR.SOURCE_ROW_2` / `SOURCE_COL_2` | `56.0` / `137.5` | 远端左标定点在原图中的像素位置。 |
| `BEV_PROJECTOR.SOURCE_ROW_3` / `SOURCE_COL_3` | `56.0` / `226.5` | 远端右标定点在原图中的像素位置。 |
| `BEV_PROJECTOR.TARGET_FORWARD_0` / `TARGET_LATERAL_0` | `0.0795685735` / `-0.2316051101` | 近端左标定点对应的车辆坐标。 |
| `BEV_PROJECTOR.TARGET_FORWARD_1` / `TARGET_LATERAL_1` | `0.0795685735` / `0.2316051101` | 近端右标定点对应的车辆坐标。 |
| `BEV_PROJECTOR.TARGET_FORWARD_2` / `TARGET_LATERAL_2` | `0.7834243479` / `-0.2316051101` | 远端左标定点对应的车辆坐标。 |
| `BEV_PROJECTOR.TARGET_FORWARD_3` / `TARGET_LATERAL_3` | `0.7834243479` / `0.2316051101` | 远端右标定点对应的车辆坐标。 |

摄像头角度变化后优先使用 `new/user/calibrate_bev_projector_from_live.py` 在直道居中静态帧上做多行边界拟合；脚本默认只输出建议和 overlay，显式 `--write-params` 才写回 `BEV_PROJECTOR.SOURCE_*`。当前米制尺度以同一原始 YUYV 中真实 `0.12m × 0.05m` 的红色矩形为基准，并按车辆坐标原点 `(0,0)` 固定变换原点，对旧 BEV 应用 `forward_new = 1.3044028437 * forward_old`、`lateral_new = 1.1028814766 * lateral_old`。因此近端 `TARGET_FORWARD_0/1` 也参与换算，不保留旧 `0.061m`。该变换作用在 `TARGET_*`，因此所有 image-to-BEV 生成的 `forward_m/lateral_m` 都统一进入真实米制；禁止在 boundary、path、tracking、Circle、ML 或 viewer 再做第二次缩放。调 `SOURCE_*` 或 `TARGET_*` 时必须重新检查 raw/BEV 显示、boundary jumps/spans 和 reference overlay。不要通过 lateral-error 或 PID 参数掩盖标定错误。

## 9. BEV Geometry 行扫描

| 参数 | 当前 JSON 值 | 作用与调参方法 |
| --- | --- | --- |
| `BEV_GEOMETRY.FORWARD_SAMPLE_0` | `0.050000000000` | reference path 第 0 层。当前视觉 reference 只能从 boundary facts 形成的近端连续候选开始。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_1` | `0.076363636364` | 第 1 层。用于 leading 连续段和插值。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_2` | `0.102727272727` | 第 2 层。默认 `MIN_LEADING_REFERENCE_SAMPLES=3` 时，这是最小 usable 远端。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_3` | `0.129090909091` | 第 3 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_4` | `0.155454545455` | 第 4 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_5` | `0.181818181818` | 第 5 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_6` | `0.208181818182` | 第 6 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_7` | `0.234545454545` | 第 7 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_8` | `0.260909090909` | 第 8 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_9` | `0.287272727273` | 第 9 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_10` | `0.313636363636` | 第 10 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_11` | `0.340000000000` | 第 11 层；前 20% 区域的唯一分界点。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_12` | `0.436666666667` | 第 12 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_13` | `0.533333333333` | 第 13 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_14` | `0.630000000000` | 第 14 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_15` | `0.726666666667` | 第 15 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_16` | `0.823333333333` | 第 16 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_17` | `0.920000000000` | 第 17 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_18` | `1.016666666667` | 第 18 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_19` | `1.113333333333` | 第 19 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_20` | `1.210000000000` | 第 20 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_21` | `1.306666666667` | 第 21 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_22` | `1.403333333333` | 第 22 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_23` | `1.500000000000` | 第 23 层；当前算法不会为了远端点跨 gap 补点。 |
| `BEV_GEOMETRY.SPARSE_ROW_COUNT` | `24` | 启用 24 个 `FORWARD_SAMPLE_*` 的前 N 行。设为 `12` 表示只扫描并输出 `FORWARD_SAMPLE_0..11`，不会把 12 行重新分布到完整的 0.05..1.5m。 |
| `BEV_GEOMETRY.SEARCH_LATERAL_LIMIT_M` | `1.764610363` | 旧虚构横向半宽 `1.6` 按横向尺度换算。它不是原图有效 span 裁剪。 |
| `BEV_GEOMETRY.LATERAL_STEP_M` | `0.022057630` | 旧虚构横向步长 `0.02` 按横向尺度换算。 |
| `BEV_GEOMETRY.BOUNDARY_TRACE_MAX_ADJACENT_DISTANCE_M` | `0.195660427` | trace 沿前向行推进，旧阈值 `0.15` 按前向尺度换算；这是等效换算值，不是新实测真值。 |
| `BEV_GEOMETRY.NOMINAL_ROAD_HALF_WIDTH_M` | `0.225` | 实测赛道全宽 `0.45m` 的半宽；覆盖旧虚构值的等效换算。CircleV2 ExitTrace 通过同一 `OrdinaryRoadModel.half_width` 消费。 |

`FORWARD_SAMPLE_*` 必须单调递增。当前 24 点包含两个端点：前 20% 物理区域 `0.05..0.34m` 含端点放置 12 点，间距为 `(0.34 - 0.05) / 11 = 0.026363636364m`；后续 `(0.34..1.50m]` 放置剩余 12 点，间距为 `(1.50 - 0.34) / 12 = 0.096666666667m`。分界点 `0.34m` 只出现一次并属于前段。这些参数位于校正后的 BEV 车辆坐标系 `forward_m`，消费方不得再缩放。改采样分布会影响 LUT identity、leading range、lateral-error 权重含义和 steering media snapshot；不要只改某一个点来修局部画面。

`SPARSE_ROW_COUNT` 是活跃前缀长度，合法范围为 `1..24`。它改变性能和最大前视距离，但不改变任何已定义采样行的物理位置；参数变化会让 sparse LUT 与 hold geometry identity 失效并重建。

同一条 sparse BEV 横线内，两个边点只有通过统一 boundary helper 形成同一 `BEVBoundarySpan` 后，才能被认为是同一道路片段的两边。图像外、不可采样或投影失败部分不形成 boundary，也不作为隔断。这个 row 内 span 事实与 row 间 boundary trace 连续性叠加使用，避免把断裂边界拼成同一道路。

`BOUNDARY_TRACE_MAX_ADJACENT_DISTANCE_M` 是边界连续性裁剪的唯一距离来源，合法值为有限正数。它的距离定义在 BEV 图/BEV 车辆坐标系内，单位是米，计算对象是 `(forward_m,lateral_m)` 点之间的 BEV metric 距离；它不是原图像素距离，也不参与原图投影关系防御。ordinary candidate 生成只读取该参数并传给 boundary trace helper；不从 `NOMINAL_ROAD_HALF_WIDTH_M`、`LATERAL_STEP_M` 或图像量化误差现场构造距离，也不额外加入量化容差。

原图到 BEV 的关系只用于采样事实：原图提供 Y 值，BEV row facts 和 BEV metric 几何负责路径判断。不要新增“原图和 BEV 是否匹配”的业务防御判断。

## 10. Illumination-compensated Binary Boundary

active runtime 只接受 `320x240 YUYV`。它把图像划分为 `20x15` 个 `16x16` 单元，读取每个单元中心的 Y，并在缩小图上执行与 Python 参考一致的 Gaussian blur（原图半径 `35px`，缩小图半径 `35/16`），形成局部 illumination `L`。随后在固定 300 个中心样本的残差 `10Y-5L` 上计算二类 Otsu，并把固定 margin `22` 加到阈值上。全图唯一分类合同为 `10Y-5L > residual_threshold` 判白，等于阈值判黑。权重 `5/10` 保留一半局部照明补偿，避免旧 `9/10` 把白路面中的轻微灰度下降放大为按 `16x16` 网格对齐的黑色阶梯。

`BinaryModelState` 同时包含 `residual_threshold` 和完整 `20x15 illumination`，不得把阈值脱离局部照明单独解释。残差直方图不能形成两个非空类别时本帧模型无效；此时最多沿用最近一份完整模型 3 帧，第 4 帧清空。稀疏行分类和原图 supercover 连通性检验共享同一对象和同一谓词；边界 owner 只消费 scanner 发布的黑白事实，不重复分类，也不存在旧全帧亮度阈值或局部亮度差回退。

相邻横向 BEV 样本发生黑白转换时产生 jump，rising/falling 配对形成 span；采样索引缺口会中断邻接，不会跨不可观测区域造边界。上位机只有在媒体帧为快照对齐、全分辨率 `gray8` 且携带完整 binary model 时才生成精确二值层。

## 11. BEV Classification 与 hold

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `BEV_CLASSIFICATION.WHITE_CONFIDENCE_MIN` | `0.55` | debug/legacy classification | active binary-model line/cross/CircleV2 不读取该值；仅保留给显式 debug/legacy/test 分类入口。不要用它调当前寻线。 |
| `BEV_CLASSIFICATION.UNKNOWN_CONFIDENCE_MIN` | `0.25` | debug/legacy classification | active binary-model line/cross/CircleV2 不读取该值；仅保留给显式 debug/legacy/test 分类入口。不要用它调当前寻线。 |
| `BEV_CLASSIFICATION.HOLD_LAST_MAX_CYCLES` | `64` | reference continuity | 当前视觉 facts 不 usable 时最多 hold 上次路径的周期数。短暂丢点可增大；不想让旧路径影响控制就减小。hold 点必须显示 `reference.mode=hold_last`、`reference.source=hold`。 |

`BEV_CLASSIFICATION` 不再是 active sparse row 二值 authority；line/cross/CircleV2 读取 binary model 派生的 boundary jumps/spans/traces 和相关 element evidence。

## 12. BEV Control Model

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `BEV_CONTROL_MODEL.LATERAL_OFFSET_TO_WHEEL_DELTA_GAIN` | `300` | turn-output target | 当前默认增益 `300`；尚未实车重调。 |
| `BEV_CONTROL_MODEL.HEADING_ERROR_TO_WHEEL_DELTA_GAIN` | `120` | turn-output target | 当前默认增益 `120`；大航向误差下需实车复核。 |
| `BEV_CONTROL_MODEL.CURVATURE_TO_WHEEL_DELTA_GAIN` | `20` | turn-output target | 当前默认增益 `20`；尚未实车重调。 |
| `BEV_CONTROL_MODEL.MIN_LEADING_REFERENCE_SAMPLES` | `3` | reference usability | 第一个连续真实 reference 点段的最小数量。近端丢线本身不使路径不可用，但真实连续点少于该值仍不可用。低于 3 时按 3 处理。 |
| `BEV_CONTROL_MODEL.TRACKING_FIT_MIN_SAMPLES` | `3` | reference tracking geometry | 二次拟合 `reference_tracking_geometry` 所需的最小 leading usable 样本数。合法范围 `[3, 24]`。 |

旧 `BEV_CONTROL_MODEL.LATERAL_ERROR_TO_WHEEL_DELTA_GAIN` 兼容别名已从 active runtime 移除。`reference_lateral_error` 只保留为固定权重的 legacy debug 对照事实，不再有 JSON 调参面。

## 13. BEV Element

Circle V2 架构见 `new/docs/visual-element-sparse-circle-v2.zh-CN.md`。运行时 circle 语义归 `CircleV2Scene` 所有；`RunVisualElementPipeline()` 只保留 cross / non-circle visual element evidence。旧 circle evidence 参数面已删除，不再作为运行时配置或媒体解释依据。

### 13.1 判定链与不可调合同

调 `BEV_ELEMENT` 前先定位第一处错误事实，不要用后续状态参数补偿上游误判：

- Cross 的当前帧 opening 有两种来源：唯一 origin-connected 白区连续至少 3 行同时到达左右 FOV；或一侧真实边界相对此前连续同侧边界基线向外扩张，且对侧为真实 FOV。外扩使用米制有向垂距，不使用道路宽度、跨帧状态或额外持续计数。多个 origin-connected 白区视为歧义，内部不可观察缺口不等于 FOV。Cross evidence 还要求 origin 到 `CROSS_CONNECTIVITY_SAMPLE_INDEX` 指定的 BEV 前向采样点中点的 supercover 连通性为 `connected`。
- Circle opening 只使用 opening ROI 内唯一一个 origin-connected white run。基线只由此前连续、真实可见的同侧边界组成；候选必须满足同侧有向垂距、连续前向确认长度和真实对侧直线三项条件。左右同时成立时不判方向。
- FOV 边缘只可作为 Circle opening 的保守距离下界；它不进入基线、Inner/Exit geometry、普通边界或控制参考。
- Circle opening/geometry 的相邻行最大距离来自 `BEV_GEOMETRY.BOUNDARY_TRACE_MAX_ADJACENT_DISTANCE_M`，不在 `BEV_ELEMENT` 内。对侧直线拟合的固定最大漂移目前约为 `0.0662m`，也不是运行参数。
- `CIRCLE_V2_INNER_TRACE_PATH_OFFSET_M` 是路径生成参数，不参与开口、方向、Entry 或 Exit 判定；`CIRCLE_V2_ENABLED` 是组合开关，也不是视觉阈值。

### 13.2 Cross 参数

| 参数 | 当前 JSON 值 | owner 与精确语义 | 增大时 | 减小时 | 当前建议与证据 |
| --- | ---: | --- | --- | --- | --- |
| `BEV_ELEMENT.CROSS_MIN_SAMPLEABLE_PER_ROW` | `8` | open row 可参与 Cross 连续游程前所需的最少可采样点数；合法值 `>=1`。它不改变固定的连续 3 行门槛。 | 行观测要求更严格，远端/FOV 较窄行更易变成 `insufficient_sampleable_support`，漏检增加。 | 允许观测支撑更少的行进入开口判定，灵敏度提高，但局部不可观测更易被当成开口。 | 保持 `8`，直到 aligned evidence 能给出误判/漏判行的逐行 `sampleable_count`。若误判行长期只有少量样本，逐级试 `10/12`；若真实十字因支撑不足被拒，逐级试 `6`，不要同时改 binary model 或连通性。 |
| `BEV_ELEMENT.CROSS_CONNECTIVITY_SAMPLE_INDEX` | `9` | Cross 连通性 owner 从固定 24 个 `BEV_GEOMETRY.forward_samples_m` 中选择第 N 点，并检验 `(0,0)` 到 `(forward_samples_m[N],0)` 的 supercover 连通性。合法值 `0..23`，且不受 `SPARSE_ROW_COUNT` 的启用前缀影响。该参数只定义 Cross 判定的前向连通性护栏。 | 护栏更深入路口，车辆存在航向偏差时更容易离开入口走廊而被阻断。 | 护栏过近时只能证明较短的入口走廊。 | 当前使用 `9`（约 `0.287m`），验证入口连通性而不把固定车体中轴延伸到路口深处。 |
| `BEV_ELEMENT.CROSS_BOUNDARY_EXPANSION_MIN_M` | `0.055` | 单侧真实边界相对此前连续同侧边界拟合直线的最小向外垂距；合法值 `(0,2]m`。至少需要两个前序真实边界点，且另一侧端点必须为真实 FOV。 | 更严格，弱外扩漏检增加。 | 更灵敏，边界抖动和渐弯更容易触发。 | 保持 `0.055m`，调参前先查看 `present_left_boundary_expansion` / `present_right_boundary_expansion` 对齐帧。 |

### 13.3 Circle opening 与 Entry 参数

| 参数 | 当前 JSON 值 | owner 与精确语义 | 增大时 | 减小时 | 当前建议与证据 |
| --- | ---: | --- | --- | --- | --- |
| `BEV_ELEMENT.CIRCLE_V2_ENABLED` | `1` | 启动期是否注册 `CircleV2Scene`。关闭不是“提高判定阈值”，而是完全移除 CircleV2 输出。 | 仅布尔开关。 | 设为 `0` 后不应再期待 Circle memory、reference 或 telemetry 正常转移。 | 保持 `1`。只在隔离实验中关闭；切换后必须重建/reset scene memory。 |
| `BEV_ELEMENT.CIRCLE_V2_OPPOSITE_STRAIGHT_CONFIDENCE_MIN` | `0.7` | opening 候选确认区间内，真实对侧边界拟合置信度的下限；合法值 `[0,1]`。 | 对侧必须更直，普通弯道误判减少，真实环岛入口因噪声/遮挡漏检增加。 | 更容易接受弯曲或抖动的对侧边界，灵敏度提高、普通弯道误判增加。 | 保持 `0.7`。当前 telemetry 只发布 `opposite_straight` 布尔值，没有原始 confidence；缺少数值证据时不建议微调。 |
| `BEV_ELEMENT.CIRCLE_V2_MIN_SAMPLEABLE_WIDTH_M` | `0.35` | 行进入 opening owner 所需的最小横向可采样宽度；合法值 `(0,10]m`。 | 排除更多窄 FOV 行，降低边缘误判，但可用行减少，基线/确认更易断裂。 | 纳入更多窄行，远端覆盖增加，但边界事实可能只来自很小视野。 | 保持 `0.35m`。若 reason 指向行支撑不足且原图确有完整边界才减小；不要用它补偿错误投影或采样缺口。 |
| `BEV_ELEMENT.CIRCLE_V2_OPENING_FORWARD_MIN_M` | `0.05` | opening 搜索 ROI 绝对近端下限；合法值 `[0,2]m` 且不得大于 max。 | 忽略更多车头附近点，减少近端透视/遮挡噪声，但缩短基线与确认区间。 | 纳入更近点；只有投影和边界在近端可靠时才有价值。 | 保持 `0.05m`，已经覆盖当前第一行。没有近端假边界证据时不提高。 |
| `BEV_ELEMENT.CIRCLE_V2_OPENING_FORWARD_MAX_M` | `1.5` | opening 搜索 ROI 绝对远端上限；合法值 `[0,2]m` 且不得小于 min。 | 纳入更远点，可能更早锁存方向，但远端分辨率、连通性和拟合误差风险上升。 | 延后方向发现并缩短历史基线；过低会在开口靠近前完全看不到 cue。 | 保持 `1.5m`，与当前最远 BEV 行一致。要抑制远端假 cue，应先用直道负样本验证 `outward_distance_m`，不要直接裁 ROI。 |
| `BEV_ELEMENT.CIRCLE_V2_OPENING_DISTANCE_MIN_M` | `0.055` | 当前同侧有效边界/FOV 下界相对此前同侧基线的最小有向垂距；合法值 `(0,2]m`。这是“是否为开口”的直接阈值。 | 更严格，抑制边界抖动/普通弯道误判，弱开口漏检增加。 | 更灵敏，小外扩也触发，直道抖动和渐弯更易误判。 | 当前保持 `0.055m`。2026-07-26 推车中 observed-boundary 中位数约 `0.0652m`、最小约 `0.0582m`，余量仅 3–10mm；若直道负样本确认误判，单变量试 `0.060m`，不要先改 Entry。 |
| `BEV_ELEMENT.CIRCLE_V2_OPENING_CONFIRM_FORWARD_SPAN_M` | `0.1` | 从 opening frontier 起，同一基线下有向垂距必须连续成立的前向长度；合法值 `(0,2]m`。不可观察行或相邻距离超限会中断。 | 要求更长持续区间，单点噪声更难触发，但近端/FOV 开口可能因可见长度不足漏检。 | 更快确认，弱化空间持续性，瞬态边界变化更易触发。 | 保持 `0.10m`。本次 FOV 有效序列最小确认长度约 `0.1055m`；直接升到 `0.12m` 会拒绝部分当前有效近端序列。只有确认存在开口闪烁误判时再单变量试 `0.12m`。 |
| `BEV_ELEMENT.CIRCLE_V2_ENTRY_FORWARD_MIN_M` | `0.1` | Approach 进入 InnerTrace 时，统一 opening frontier 必须达到的近端下限；合法值 `[0,2]m` 且不得大于 max。它不参与 opening 检测。 | 要求 frontier 不得过近，可能错过已经贴近车头的开口。 | 允许更近才进入，对当前非等距采样通常只扩大近端容错。 | 保持 `0.10m`。当前有效 Entry frontier 远高于该值，没有下限造成拒绝的证据。 |
| `BEV_ELEMENT.CIRCLE_V2_ENTRY_FORWARD_MAX_M` | `0.5` | Approach 进入 InnerTrace 时，统一 opening frontier 的远端上限；合法值 `[0,2]m` 且不得小于 min。它只决定何时切状态。 | 更早进入 InnerTrace，但过大时远端 observed-boundary cue 也可能提前接管。 | 必须等开口更靠近，进入更晚；车辆速度高时可能错过切换窗口。 | 默认仍为 `0.50m`；下一轮单变量建议试 `0.55m`。本次 frontier 从 `0.533m` 跳到 `0.437m` 后才进入，`0.55m` 只跨过一个离散采样档；暂不建议 `>=0.65m`。 |

### 13.4 Circle 状态、几何与路径参数

| 参数 | 当前 JSON 值 | owner 与精确语义 | 增大时 | 减小时 | 当前建议与证据 |
| --- | ---: | --- | --- | --- | --- |
| `BEV_ELEMENT.CIRCLE_V2_NORMAL_TRACE_START_YAW_DEG` | `90` | 从进入 InnerTrace 的 yaw 原点起，方向归一化的历史最大 directed yaw 达到 X 后进入 NormalTrace。NormalTrace 不发布 Circle reference，恢复普通双边/单边寻线。 | 延长仅寻内线阶段。 | 更早恢复普通寻线。 | `90deg` 是本次状态语义的初始假设；必须用完整动态绕环证据校准。 |
| `BEV_ELEMENT.CIRCLE_V2_EXIT_TRACE_START_YAW_DEG` | `270` | 同一 yaw 事实达到 Y 后进入 ExitTrace，仅发布外线参考；必须满足 `X < Y < Z`。 | 延后仅寻外线。 | 更早切到外线。 | `270deg` 是初始假设，不由静态 Y 点图片校准。 |
| `BEV_ELEMENT.CIRCLE_V2_CALM_FALLBACK_YAW_DEG` | `340` | 达到 Z 时无条件从角度阶段进入 CalmTrace；真实外线在 ExitTrace 被观察到也可提前进入 CalmTrace。 | 延后保底退出。 | 更早保底退出。 | `340deg` 是保底初值；固定出环射线属于 Circle 私有推断几何，不能触发该提前转移。 |
| `BEV_ELEMENT.CIRCLE_V2_CALM_TRACE_MS` | `1000` | CalmTrace 按 capture time 持续寻外线的时间，结束后进入 Cooldown。 | 出环后保持外线更久。 | 更早交还普通寻线。 | 初值 `1000ms`；以动态出环轨迹校准。 |
| `BEV_ELEMENT.CIRCLE_V2_COOLDOWN_MS` | `3000` | Cooldown 按 capture time 屏蔽新 Circle 入口，期间不发布 Circle reference；结束回 Idle。 | 更不易重复进环，但下一环岛识别更晚。 | 更快允许再次进入。 | 初值 `3000ms`。 |
| `BEV_ELEMENT.CIRCLE_V2_INNER_TRACE_STALL_TIMEOUT_MS` | `2000` | InnerTrace 已持续至少该时间且历史最大 directed yaw 仍小于 stall yaw 门槛时，退回 Idle；合法值 `>=1ms`。 | 给慢速/起步更多时间，错误 InnerTrace 也会滞留更久。 | 更快清除假进入，但低速车辆可能尚未积累足够 yaw 就被退出。 | 保持 `2000ms`。调该值必须同时读取 `inner_trace_elapsed_ms` 和 `directed_turn_angle_rad`；不要只看最终 phase。 |
| `BEV_ELEMENT.CIRCLE_V2_INNER_TRACE_STALL_YAW_MIN_DEG` | `60` | stall timeout 到达后，“仍未明显转弯”的 directed yaw 上限；条件是 `progress < threshold`。合法值 `[0,720]deg`。 | 更容易满足 stall 条件并退回 Idle；这与“容忍更多 yaw”直觉相反。 | 只有更小 yaw 才退回，错误 InnerTrace 更难被清除；设为 `0` 时非负 progress 不会因该条件退出。 | 保持 `60deg`。若真实低速入环在 2s 内达不到 60deg，应优先增加 timeout，而不是降低此值掩盖时序问题。 |
| `BEV_ELEMENT.CIRCLE_V2_INNER_TRACE_PATH_OFFSET_M` | `0.0` | InnerTrace 路径从真实内圆边线向道路内部的偏移；合法值 `[0,2]m`，不参与任何 Circle 判定。正值左环岛向右偏、右环岛向左偏。 | 路径离内圆边线更远。 | 更贴近内圆边线。 | 保持 `0.0m` 直到动态轨迹证明偏内/偏外；之后以 `0.01m` 单步调整，并同时看实际轨迹与 control snapshot。 |
| `BEV_ELEMENT.CIRCLE_V2_INNER_GEOMETRY_FORWARD_MIN_M` | `0.05` | InnerTrace 只从该绝对前向距离起收集唯一 origin-connected run 的真实内侧边界；合法值 `[0,2]m` 且不得大于 max。 | 排除更多近端噪声，但可用点减少、首段可能后移。 | 纳入更近真实边界，前提是近端投影可靠。 | 保持 `0.05m`。geometry 至少需要 2 个连续真实点；若 unavailable，应先查端点状态和邻接缺口。 |
| `BEV_ELEMENT.CIRCLE_V2_INNER_GEOMETRY_FORWARD_MAX_M` | `0.5` | InnerTrace 真实内侧边界几何 ROI 远端上限。 | 使用更多远端点，几何可用率可能提高，但可能混入入口/非局部曲率。 | 参考更局部，点数减少并可能低于两点数学下限。 | 保持 `0.50m`；只在逐帧 geometry 点证据表明远端污染或点数不足时调整，建议步长 `0.05m`。 |
| `BEV_ELEMENT.CIRCLE_V2_EXIT_GEOMETRY_FORWARD_MIN_M` | `0.05` | ExitTrace 从该距离起收集真实对侧边界。 | 排除近端噪声，但直线点减少。 | 纳入更多近端点，可能受车头附近透视/遮挡影响。 | 保持 `0.05m`，等待真实 ExitTrace aligned evidence。 |
| `BEV_ELEMENT.CIRCLE_V2_EXIT_GEOMETRY_FORWARD_MAX_M` | `0.5` | ExitTrace 真实对侧边界几何 ROI 远端上限。 | 增加直线判定范围和点数，也更容易累计弯曲横向跨度。 | 判定更局部，可能只剩不足两点。 | 保持 `0.50m`；应与 straight span 一起观察但一次只改一个参数。 |
| `BEV_ELEMENT.CIRCLE_V2_EXIT_STRAIGHT_MAX_LATERAL_SPAN_M` | `0.13` | Exit ROI 内真实对侧边界 `max(lateral)-min(lateral)` 的上限；至少需要 2 个连续真实点。合法值 `(0,2]m`。 | Exit geometry 更宽松，弯曲边界也可能被视为直线。 | 更严格，出口抖动/轻微弯曲会使 geometry unavailable。 | 保持 `0.13m`。若真实出口持续 `geometry_available=false`，先核对点数、端点真实性和实际 lateral span；只有 span 略超阈值时才以 `0.01m` 步长增加。 |

### 13.5 推荐调参顺序

1. 先用 raw gray8 与同帧 binary model/jump/white-run 证明上游边界事实正确。
2. Circle 不触发时依次看 `origin_connected`、`opposite_straight`、`source`、`outward_distance_m`、`confirmed_forward_span_m`；只调整第一处不满足的 owner 参数。
3. opening 已正确但状态切换早/晚，只调 Entry ROI；不要反向改变 opening distance。
4. InnerTrace 已进入但参考不可用，只调 geometry ROI 或修复真实边界事实；不要放宽 opening。
5. 动态完整绕环后才能调 X/Y/Z、Calm/Cooldown、stall 和 ExitTrace 参数。路径偏移最后调，并与判定参数分轮验证。
6. 每轮只改一个责任相同的参数，保存 JSON/config snapshot、逐帧 telemetry 和 aligned raw frame；恢复默认值时以本表“当前 JSON 值”为准。

`cross_exit` 当前只发布检测证据，不生成 visual-reference candidate，也不改变普通路径、CircleV2、actuator、yaw、safety 或 hold。现场先在 no-motion capture 中确认 `element_evidence.cross_exit.{present,reason,sampleable_count,boundary_jump_count,boundary_span_count,boundary_absent_row_count}` 与 raw/BEV 画面对齐。新的 Cross 路径规划应由独立 owner 消费这份证据。

full BEV element raster 不属于 active `default_params.json` 运行时合同；需要 full raster 的 legacy/probe 测试必须通过本地显式 `BEVElementRasterParameters` 传入开关和宽度，不能从 runtime 参数或 media config snapshot 反向获取。

## 14. 禁止使用历史参数思路调车

不属于当前 `default_params.json` 的历史参数名、历史场景名、历史拓扑/策略字段，都不得作为 active 调参依据。需要查历史上下文时看 archive 文档；新的调参记录只写本文列出的当前参数和当前分层证据字段。

## 15. 改参记录模板

每次赛道改参至少记录：

```text
时间:
参数文件/commit:
只改的参数:
改参假设:
验证命令:
证据目录:
关键字段:
  perception_health:
  reference:
  eligibility:
  lateral_error:
  reference_tracking_geometry:
  reference_control:
  safety_gate:
  yaw_control:
  actuator:
结论:
下一步:
```

运行时分层与 include 边界见 `new/code/port/README.md`。`PerceptionResult is a runtime transport snapshot, not a dependency shortcut.`

后续在 `bev-simple-reference-extension` 上扩展 BEV 元素或路径策略前，先遵守根目录 `README.md` 中的大道至简与互不知晓约束。
## ML startup parameters

`MOTION_ODOMETRY.ENCODER_TICKS_TO_METER` is the single startup scale used by
both reference time alignment and the ML maneuver odometry path.
`ML.ENABLED` owns recognition scheduling. `ML.MANEUVER.ENABLED` independently
owns whether accepted recognition facts may produce a path candidate and ML
speed selection. `ML.ENABLED=1` with `ML.MANEUVER.ENABLED=0` is observe-only:
detector, ROI, classifier, mapping, timing, and artifact facts are published,
while maneuver memory is reset and no ML path or speed override is produced.
`ML.MANEUVER.ENABLED=1` with `ML.ENABLED=0` is invalid.

<!-- contract:ml-enabled-speed-target range=(0,5000] -->

When `ML.ENABLED=1`, startup validation is fail-closed. The ROI search span,
forward/lateral grid steps, expected rectangle edges and their tolerances must be finite and
positive. YUV intervals must be ordered inside `[0,255]`; orientation must be
in `(0, pi/2]`; component count must be at least one; rectangularity and red
fill thresholds must be in `(0,1]`; score weights must be finite and
nonnegative with a positive sum. Only when `ML.MANEUVER.ENABLED=1`,
`ML.MANEUVER.SPEED_TARGET` must be in `(0,5000]`,
`MOTION_ODOMETRY.ENCODER_TICKS_TO_METER` and `ML.MANEUVER.EXIT_FORWARD_M` must
be positive, and the duration and integration-gap limits must be at least 1 ms.
`ML.MANEUVER.PATH_OUTWARD_OFFSET_M` is the metric offset from the selected
observed boundary toward the road exterior (left is negative lateral, right is
positive lateral) and must be in `[0,2]`. Cooldown may be zero. While a maneuver
is active, missing current boundary samples leaves ML ownership active so the
global reference-continuity policy can HOLD the last ML path; it does not end
the maneuver. Completion is distance or time, whichever occurs first.

`ML.V9` accepts `MIN_MARGIN>=0`, `MAX_BEST_DISTANCE` in `[0,126]`, and
`CONFIRM_FRAMES>=1`. `ML.CLASS_MAPPING` is separate from acceptance. Each class
maps to one of `straight`, `left`, `right`, or `unmapped`; defaults are class 0
straight, class 1 left, and class 2 right. Duplicate mappings are permitted.

`ML.TFLITE_IDENTITY` is an independent startup-only acceptance and confirmation
contract for the d4 identity backend. Its `MIN_MARGIN` and
`MAX_BEST_DISTANCE` are squared-L2 values over four signed-int8 coordinates,
so each validates in `[0,260100]` (`4*255^2`); `CONFIRM_FRAMES` must be at least
one. The frozen artifact calibration sets `MIN_MARGIN=1`,
`MAX_BEST_DISTANCE=2076`, and `CONFIRM_FRAMES=3`: the supplied 6688-sample NPZ
has minimum correct margin 1 and maximum correct winning distance 2076, while
1226 correct samples exceed the unrelated V9 Hamming ceiling 126. Scene policy
selects this group explicitly for `tflite_int8` and continues selecting
`ML.V9` for `v9_hamming`; the two distance units are never shared.

The calibrated default uses `CONFIRM_FRAMES=3`: the current live vehicle run
started with nine consecutive class-1 frames and therefore locks vehicle on
frame 3 before later isolated class-2 noise; the scene already stops inference
after this lock, so this is the intended confirmation contract rather than a
post-classification remap.

The current ML ROI calibration consumes the corrected BEV
metric directly. Its marker truth is `EXPECTED_LONG_EDGE_M=0.120` and
`EXPECTED_SHORT_EDGE_M=0.050`; its search window was transformed with the same
projector scale about vehicle origin `(0,0)` to
`forward=[0.1304402844,0.2739245972]m` and
`lateral=+-0.1433745920m`. The old isotropic fictitious `GRID_STEP_M=0.003`
was removed because one value cannot represent two different axis scales;
the axes remain independently metric. The active detector uses a 1.2x coarser
sampling lattice, `GRID_FORWARD_STEP_M=0.0046958502` and
`GRID_LATERAL_STEP_M=0.0039703733`, to reduce per-frame work while retaining
roughly 26 by 13 samples across the calibrated 120mm by 50mm marker. These are
sampling intervals, not ML-only scale compensation.
The calibrated red interval is `Y=[45,230]`, `U=[70,135]`, and `V=[140,220]`.
It covers the measured illuminated marker envelope (`Y=[61,222]`,
`U=[75,126]`, `V=[145,210]`) while retaining chroma bounds that reject the
white paper and non-red search-area background. Rectangle dimensions,
component size, rectangularity, fill ratio, and orientation remain mandatory
acceptance gates; color alone never emits an ML rectangle. The long edge may
rotate by at most `0.7853982rad` (`45deg`, rounded outward to the detector's
`float` angle representation) from the vehicle lateral
axis, symmetrically in either direction.
The classifier crop keeps the calibrated `0.120m` square size and applies two
explicit marker-frame registration offsets: `CROP_LONG_OFFSET_M=0.0` and
`CROP_FORWARD_OFFSET_M=-0.00375`. They move the sampled square by one 32x32
pixel toward the marker with no lateral/long-axis bias. These values were
selected from a joint replay over 16 earlier, 12 current, and 16 subsequent
live-board ROIs: this registration classified `43/44` as vehicle and all
`16/16` most recent live ROIs, whereas the unregistered crop crossed the
vehicle/weapon boundary. They are
crop registration, not a class threshold or model/mapping change.

CircleV2's active metric constants were converted at their owner: the center
sample forward gap is `0.130440284m`, lateral opening threshold is
`0.055144074m`, opposite-side lateral drift limit is `0.066172889m`, and exit
trace lateral-span limit is `0.132345777m`. These are transformed equivalents
of the old fictitious values, not new physical measurements.
