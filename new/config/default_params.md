# `default_params.json` 调参指南

本文只描述当前 active runtime 仍会读取和发布的参数。旧复杂寻线、元素识别、拓扑、roadblock 占位和历史 PID 命名已归档，不再作为运行时、协议或调参依据。

当前闭环固定为：

```text
frame -> sparse BEV boundary facts -> visual reference facts -> reference usability -> tracking geometry
-> reference-control readiness -> safety gate -> yaw-control terms -> actuator
```

`new/config/default_params.json` 是人工编辑的运行默认合同。`RuntimeParameters` 内建默认值只用于缺文件或解析失败时的 fallback 镜像，必须通过 `run_runtime_parameter_defaults_test.sh` 保持同步。

## 1. 调参前提

每次只调整一个明确参数族，并先写清楚要验证的假设。不要把曝光、BEV 标定、reference usability、lateral-error、yaw PID、轮速 PID 和运动状态机混在一次修改里。

推荐现场闭环：

```bash
cd new/user
rtk ./debug.sh assistant on 192.168.137.1 39011 39012
rtk ./start_with_upload.sh no-motion
rtk env LS2K_HOST_CAPTURE_BACKEND=windows ./debug.sh steering host-capture --listen-host 0.0.0.0 --listen-port 39011 --media-listen-port 39012 --duration-s 20
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
rtk bash new/verification/tests/run_runtime_parameter_defaults_test.sh
rtk bash new/verification/tests/run_power_adapter_threshold_test.sh
rtk bash new/verification/tests/run_startup_low_voltage_order_test.sh
rtk bash new/verification/tests/run_bev_simple_perception_test.sh
rtk bash new/verification/tests/run_visual_reference_orchestration_test.sh
rtk bash new/verification/tests/run_reference_usability_lateral_error_test.sh
rtk bash new/verification/tests/run_assistant_telemetry_selftest.sh
rtk bash new/verification/tests/run_steering_media_selftest.sh
rtk bash new/verification/tests/run_perf_counter_test.sh
rtk bash new/verification/tests/run_bev_simple_residual_check.sh
```

## 2. 加载语义

运行时由 `new/code/platform/param_store.cpp` 读取 `new/config/default_params.json`。

- 缺文件：回退到内建默认值，并发布 `params.missing`。
- JSON 非法、必填字段缺失、字段类型错误：回退到内建默认值，并发布 `params.parse`。
- `exp_light` 是启动关键字段，必须在 `0..2500`；非法时触发 fail-safe。
- `exp_light != 65` 允许加载，但会发布曝光告警；当前相机基线仍以 `65` 为默认。

当前必填键：

- `RUNNING_SPEED_TARGET`
- `YAW_RATE_PID.D`
- `exp_light`
- `LEFT_WHEEL_PID.{P,I,D,INTEGRAL_LIMIT}`
- `RIGHT_WHEEL_PID.{P,I,D,INTEGRAL_LIMIT}`
- `assistant_tcp.{host,port}`

其余键都是可选覆盖；缺省时使用内建默认值。可选键格式错误仍会触发整体 parse fallback。

## 3. 诊断到参数的顺序

1. 没有 host 连接或数据很少：先看 `assistant_tcp.*`、`assistant_enabled`、`steering_media_*`，再看板端 `assistant.backoff`、`steering_media.backoff`、`steering_media.summary`。
2. 边界事实不对：先看 `exp_light`、`BEV_PROJECTOR`、`BEV_GEOMETRY`、`BEV_BOUNDARY`，并对照 raw/BEV 图像和 `boundary_jump_count/boundary_span_count`。
3. 边界事实对但 `eligibility.usable=false`：看 hold 周期 `BEV_CLASSIFICATION.HOLD_LAST_MAX_CYCLES`、`BEV_CONTROL_MODEL.MIN_LEADING_REFERENCE_SAMPLES`、`BEV_GEOMETRY.FORWARD_SAMPLE_*`。
4. reference 对但 `reference_tracking_geometry` 不合理：看 leading reference path、boundary spans/traces、`BEV_CONTROL_MODEL.TRACKING_FIT_MIN_SAMPLES`，并用 `lateral_error` 只做迁移期对照。
5. tracking geometry 合理但转向幅度不对：看 `BEV_CONTROL_MODEL.LATERAL_OFFSET_TO_WHEEL_DELTA_GAIN`、`BEV_CONTROL_MODEL.HEADING_ERROR_TO_WHEEL_DELTA_GAIN`、`BEV_CONTROL_MODEL.CURVATURE_TO_WHEEL_DELTA_GAIN`、`YAW_RATE_PID.*`、`raw_turn_output_limit`、`wheel_turn_accel_delta_scale`、`wheel_turn_decel_delta_scale`。
6. `element_evidence.cross_exit` 与画面不一致：先看连续 boundary absence、`element_evidence.cross_exit.{boundary_jump_count,boundary_span_count,boundary_absent_row_count,reason}` 和 `BEV_ELEMENT.CROSS_EXIT_TAKEOVER_ENABLED`。
7. 直行速度或左右轮跟随不对：看 `RUNNING_SPEED_TARGET`、`LEFT_WHEEL_PID.*`、`RIGHT_WHEEL_PID.*`。
8. 起步、停止、fail-safe 恢复节奏不对：看 `motion_*`、`pwm_limit`、`pwm_floor`、反转保护和低电压参数。

## 4. 速度、yaw 和轮速 PID

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `RUNNING_SPEED_TARGET` | `400.0` | motion supervisor / yaw speed scale | 运行轮速目标单位，不是 m/s。增大后车速更高，yaw target 也会按 speed scale 变化。看 `effective_speed_target`、左右 `*_speed_target`、encoder measured。先用低值确认闭环再上调。 |
| `YAW_RATE_PID.P` | `0.0` | gyro feedback | gyro yaw-rate 对 turn-output 的反馈修正增益。它不承担 reference tracking geometry 前馈/反馈幅度；摆动或 raw turn 频繁反向时先看它，单纯欠转先看 BEV control model 的三项 gain。 |
| `YAW_RATE_PID.I` | `0.0` | gyro feedback | gyro 反馈积分。当前默认不用。只有长期同向 gyro 偏差且 P/D 不能解决时小幅增加；积分过大会拖尾。 |
| `YAW_RATE_PID.D` | `0.0` | gyro feedback | 抑制 gyro 反馈误差变化。抖动和过冲明显时增加；过大时转向变钝。 |
| `LEFT_WHEEL_PID.P` | `84.0` | 左轮速度 PID | 左轮速度误差主增益。左轮跟随慢增大；PWM 抖或超调减小。看 `left_speed_target`、`left_measured_speed`、`left_drive_pwm_command`。 |
| `LEFT_WHEEL_PID.I` | `2.4` | 左轮速度 PID | 左轮长期误差积分。稳态低于目标时增大；起步后拖尾或积累过冲时减小。 |
| `LEFT_WHEEL_PID.D` | `0.75` | 左轮速度 PID | 左轮速度变化阻尼。速度抖动可增大；响应迟钝可减小。 |
| `LEFT_WHEEL_PID.INTEGRAL_LIMIT` | `100.0` | 左轮速度 PID | 左轮积分上限。积分饱和导致恢复慢时减小；长期负载跟不上且 I 有效时可增大。 |
| `LEFT_WHEEL_PID.MEASUREMENT_FILTER_ALPHA` | `0.4` | 左轮速度测量滤波 | 越大越信当前测量，响应快但噪声多；越小越平滑但滞后。看 measured speed 噪声和 PWM 震荡。 |
| `RIGHT_WHEEL_PID.P` | `96.0` | 右轮速度 PID | 右轮速度误差主增益，方法同左轮。左右默认不同，不要为了对称而强行改成一样。 |
| `RIGHT_WHEEL_PID.I` | `2.2` | 右轮速度 PID | 右轮长期误差积分，方法同左轮。 |
| `RIGHT_WHEEL_PID.D` | `0.2` | 右轮速度 PID | 右轮速度变化阻尼，方法同左轮。 |
| `RIGHT_WHEEL_PID.INTEGRAL_LIMIT` | `100.0` | 右轮速度 PID | 右轮积分上限，方法同左轮。 |
| `RIGHT_WHEEL_PID.MEASUREMENT_FILTER_ALPHA` | `0.4` | 右轮速度测量滤波 | 右轮测量滤波，方法同左轮。 |

## 5. 相机与基础时序

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `exp_light` | `65` | camera startup critical | 曝光/亮度基线。边界跳变整体不足可上调，背景纹理或高光产生过多边界时下调。改动后必须看原始 raw/BEV、`boundary_jump_count`、`boundary_span_count` 和 `reference`；不要用控制结果倒推曝光。 |
| `CAMERA_SOURCE.BACKEND` | `v4l2_yuyv` | camera capture worker | 主相机源。默认直接走 V4L2 YUYV，避免 supplier MJPG/OpenCV 转换进入 foreground perception path。 |
| `CAMERA_SOURCE.DEVICE` | `/dev/video0` | camera frame source | V4L2 设备路径。换摄像头设备名时只改这里。 |
| `CAMERA_SOURCE.WIDTH` / `HEIGHT` | `320` / `240` | camera frame source | source 输出几何，必须不超过编译期 frame storage。 |
| `CAMERA_SOURCE.FPS` | `60` | camera frame source | 请求帧率；driver 可能协商失败，实际以 camera source health/perf 为准。 |
| `CAMERA_SOURCE.BUFFER_COUNT` | `3` | camera frame source | V4L2 mmap buffer 数。过小容易丢帧，过大可能增加队列滞后。 |
| `CAMERA_SOURCE.POLL_TIMEOUT_MS` | `50` | camera capture worker | capture thread 内等待上限；不阻塞 main/control loop。 |
| `CAMERA_SOURCE.DRAIN_READY_BUFFERS` | `1` | camera frame source | 一次 wait 中 drain 已就绪 buffer，减少 backend queue 旧帧。latest 仍只属于 Frame Store。 |
| `CAMERA_SOURCE.FALLBACK_BACKEND` | `vendor_uvc` | camera frame source | V4L2 startup 失败时的 supplier fallback；fallback 仍被包在 frame source 边界内。 |
| `control_period_ms` | `5` | control timer | 控制 tick 周期。减小会提高 CPU/IO 压力；增大会降低控制响应。看 perf、`control.tick` 和实际电机稳定性。 |
| `perception_stale_ms` | `120` | safety gate | 最新 perception 超过该时间即 stale。摄像头偶发慢帧可适当增大；过大则会让旧 reference 继续影响控制。看 `safety_gate.reason=perception_stale`。 |
| `control_snapshot_emit_interval_ms` | `100` | debug reporter | 板端 `control.snapshot` 与 `control.steering_snapshot` 输出周期。只影响日志密度，不改变控制。 |

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
这些组合不满足时配置加载回退默认参数，而不是等 estimator 运行时再 fail closed。

## 6. 执行器与运动状态机

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `pwm_limit` | `5000` | actuator safety | 左右轮 PWM 绝对限幅。车无力且 PID 未饱和时不要先改它；只有确认输出长期被限幅且硬件允许时上调。 |
| `raw_turn_output_limit` | `20000` | turn output safety | turn-output 绝对限幅，单位与左右轮速半差一致。它是兜底边界，不是常规转向幅度调参旋钮；满幅时目标大约是 `speed ± raw_turn_output_limit`。 |
| `wheel_turn_accel_delta_scale` | `2.0` | wheel target mixer | 差速混合中加速侧 turn delta 缩放系数。默认 `1.0` 保持旧行为；合法值为有限非负数，不设上限。正 turn 时左轮使用该系数，负 turn 时右轮使用该系数。 |
| `wheel_turn_decel_delta_scale` | `1.0` | wheel target mixer | 差速混合中减速侧 turn delta 缩放系数。默认 `1.0` 保持旧行为；合法值为有限非负数，不设上限。正 turn 时右轮使用该系数，负 turn 时左轮使用该系数；减速侧目标低于 0 时仍 clamp 到 0。 |
| `pwm_floor` | `0` | actuator shaping | 非零 PWM 的最小地板。低速克服静摩擦可小幅上调；过高会让轻微控制也变成突跳。 |
| `prohibit_reverse_pwm` | `0` | actuator safety | 禁止输出反向 PWM。需要禁止反向时显式开启；关闭会扩大硬件风险。 |
| `prohibit_reverse_pwm_step_limit` | `1000` | actuator safety | 反转保护/输出变化步进限制。反向突变风险高时减小；输出响应太慢且无反向风险时增大。 |
| `brushless_debug_fixed_pwm_enabled` | `1` | actuator debug | 启用后，正常可驱动周期把左右无刷电调命令固定为 `brushless_debug_fixed_pwm`；关闭后左右无刷电调命令为 `0`。 |
| `brushless_debug_fixed_pwm` | `600` | actuator debug | 无刷电调固定调试 PWM，合法范围 `[0, 1000]`。该值只进入统一 `ActuatorCommand` 的左右无刷字段，不在 adapter/bridge 内隐藏生成。 |
| `motion_unveto_confirm_cycles` | `3` | motion supervisor | safety gate 解除后需要连续干净周期数。误解除风险高时增大；恢复太慢时减小。 |
| `motion_spinup_ms` | `800` | motion supervisor | 起步速度爬升时间。起步打滑或冲击大时增大；起步太慢时减小。 |
| `motion_turn_limit_spinup` | `1.0` | motion supervisor | 起步阶段转向限幅比例。起步时转向过猛减小；起步弯道跟不上增大。 |
| `motion_pwm_step_limit` | `3000` | motion supervisor | motion 阶段 PWM 步进限制。输出突变大时减小；响应太慢时增大。 |
| `motion_stop_ms` | `300` | motion supervisor | stop 阶段速度衰减时间。停车太急增大；停车拖尾减小。 |
| `motion_stop_encoder_threshold` | `8` | motion supervisor | 判定停止的 encoder 阈值。车已停但不退出 STOPPING 可增大；未停就退出可减小。 |
| `motion_fault_rearm_hold_ms` | `600` | motion supervisor | fail-safe latch 后允许 rearm 前的保持时间。现场排障保守时增大；恢复流程过慢时减小。 |

## 7. Low Voltage 与调试传输

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `low_voltage_raw_threshold` | `200` | power adapter / safety gate | ADC raw 低电压阈值。实际使用值记录在 `LowVoltageSample.threshold`；`LS2K_LOW_VOLTAGE_RAW_THRESHOLD` 环境变量优先。误报低电压时先查 ADC raw，再谨慎下调；不设上限，超大正数会更保守。 |
| `low_voltage_sample_interval_ms` | `1000` | low-voltage sampler | 运行期低电压采样周期。默认 1Hz；降低会增加 IO，升高会降低低电压发现速度。 |
| `assistant_enabled` | `1` | assistant TCP | 是否启用 command/ACK/telemetry 链路。连接调试时保持开启；纯离线运行可关闭。 |
| `assistant_tcp.host` | `192.168.137.1` | assistant TCP | 板端主动连接的 host 地址。Windows 热点链路通常是 `192.168.137.1`；错误时板端会 `assistant.backoff Connection refused/timeout`。 |
| `assistant_tcp.port` | `48011` | assistant TCP | host assistant listener 端口。必须和 `debug.sh assistant on/local` / `debug.sh steering host-capture` / `tune_speed.py` 一致。Windows 热点链路优先使用高端口，避免低端口被系统策略拒绝绑定。 |
| `steering_media_enabled` | `1` | steering media TCP | 是否启用图像和 steering snapshot side channel。调视觉/boundary facts 时保持开启；带宽或 CPU 排查时可临时关闭。 |
| `steering_media_port` | `48012` | steering media TCP | host media listener 端口。必须和 `--media-listen-port` 一致。 |
| `steering_media_publish_interval_ms` | `20` | steering media service | 图像发布间隔。`20ms` 理论上约 `50fps`；实际看 host `effective_fps` 和板端 `steering_media.summary.skip_interval/image_sent/image_queued`。弱热点链路优先降位深或降采样，确认队列不堆积后再压低该间隔。 |
| `steering_media_downsample` | `1` | steering media service | 图像 side channel 的发送降采样倍率。`1` 保留 320x240 显示尺寸；热点链路吞吐不足时可临时设为 `2`/`4`，header 仍保留 source 尺寸和 downsample。 |
| `steering_media_gray_bits` | `2` | steering media service | 图像传输灰度位深。支持 `1/2/4/8`。`2` 使用 `gray2_packed`，320x240 固定四分之一带宽；距离更远或热点吞吐不足时用 `1`，需要更清晰实时画面时用 `4`，需要原始 gray8 证据时设为 `8` 或用 `--media-gray-bits 8`。 |
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
| `BEV_GEOMETRY.FORWARD_SAMPLE_0` | `0.130440284` | reference path 第 0 层。当前视觉 reference 只能从 boundary facts 形成的近端连续候选开始。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_1` | `0.215509525` | 第 1 层。用于 leading 连续段和插值。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_2` | `0.300580069` | 第 2 层。默认 `MIN_LEADING_REFERENCE_SAMPLES=3` 时，这是最小 usable 远端。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_3` | `0.385649310` | 第 3 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_4` | `0.470719854` | 第 4 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_5` | `0.555789094` | 第 5 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_6` | `0.640858335` | 第 6 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_7` | `0.725928879` | 第 7 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_8` | `0.810998120` | 第 8 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_9` | `0.896068664` | 第 9 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_10` | `0.981137905` | 第 10 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_11` | `1.066207145` | 第 11 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_12` | `1.151277689` | 第 12 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_13` | `1.236346930` | 第 13 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_14` | `1.321416170` | 第 14 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_15` | `1.406486715` | 第 15 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_16` | `1.491555955` | 第 16 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_17` | `1.576626500` | 第 17 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_18` | `1.661695740` | 第 18 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_19` | `1.746764980` | 第 19 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_20` | `1.831835525` | 第 20 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_21` | `1.916904765` | 第 21 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_22` | `2.001975310` | 第 22 层。 |
| `BEV_GEOMETRY.FORWARD_SAMPLE_23` | `2.087044550` | 第 23 层；当前算法不会为了远端点跨 gap 补点。 |
| `BEV_GEOMETRY.SPARSE_ROW_COUNT` | `24` | 启用原 24 个 `FORWARD_SAMPLE_*` 的前 N 行。设为 `12` 表示只扫描并输出 `FORWARD_SAMPLE_0..11`，不是把 12 行重新均匀分布到 0.061..1.5m。 |
| `BEV_GEOMETRY.SEARCH_LATERAL_LIMIT_M` | `1.764610363` | 旧虚构横向半宽 `1.6` 按横向尺度换算。它不是原图有效 span 裁剪。 |
| `BEV_GEOMETRY.LATERAL_STEP_M` | `0.022057630` | 旧虚构横向步长 `0.02` 按横向尺度换算。 |
| `BEV_GEOMETRY.BOUNDARY_TRACE_MAX_ADJACENT_DISTANCE_M` | `0.195660427` | trace 沿前向行推进，旧阈值 `0.15` 按前向尺度换算；这是等效换算值，不是新实测真值。 |
| `BEV_GEOMETRY.NOMINAL_ROAD_HALF_WIDTH_M` | `0.225` | 实测赛道全宽 `0.45m` 的半宽；覆盖旧虚构值的等效换算。CircleV2 ExitTrace 通过同一 `OrdinaryRoadModel.half_width` 消费。 |

`FORWARD_SAMPLE_*` 必须单调递增。旧虚构坐标的 0.1..1.6 已整体乘前向尺度，当前 24 点按 0.130440284..2.087044550m 分布，步长约 0.085069m。这些参数已经是校正后的 BEV 车辆坐标系 `forward_m`，消费方不得再缩放。改采样分布会影响 LUT identity、leading range、lateral-error 权重含义和 steering media snapshot；不要只改某一个点来修局部画面。

`SPARSE_ROW_COUNT` 是活跃前缀长度，合法范围为 `1..24`。它改变性能和最大前视距离，但不改变任何已定义采样行的物理位置；参数变化会让 sparse LUT 与 hold geometry identity 失效并重建。

同一条 sparse BEV 横线内，两个边点只有通过统一 boundary helper 形成同一 `BEVBoundarySpan` 后，才能被认为是同一道路片段的两边。图像外、不可采样或投影失败部分不形成 boundary，也不作为隔断。这个 row 内 span 事实与 row 间 boundary trace 连续性叠加使用，避免把断裂边界拼成同一道路。

`BOUNDARY_TRACE_MAX_ADJACENT_DISTANCE_M` 是边界连续性裁剪的唯一距离来源，合法值为有限正数。它的距离定义在 BEV 图/BEV 车辆坐标系内，单位是米，计算对象是 `(forward_m,lateral_m)` 点之间的 BEV metric 距离；它不是原图像素距离，也不参与原图投影关系防御。ordinary candidate 生成只读取该参数并传给 boundary trace helper；不从 `NOMINAL_ROAD_HALF_WIDTH_M`、`LATERAL_STEP_M` 或图像量化误差现场构造距离，也不额外加入量化容差。

原图到 BEV 的关系只用于采样事实：原图提供 Y 值，BEV row facts 和 BEV metric 几何负责路径判断。不要新增“原图和 BEV 是否匹配”的业务防御判断。

## 10. BEV Boundary

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `BEV_BOUNDARY.LOCAL_JUMP_MIN_Y` | `32` | sparse BEV row boundary facts | 同一条 sparse BEV 横线上相邻 Y sample 形成边界跳变所需的最小亮度差。调参只看板端发布的 `boundary_jump_count`、`boundary_span_count`、row facts 与 raw/BEV 画面对齐情况。 |

`BEV_BOUNDARY` 是 V9 active 感知入口；line/cross/CircleV2 只消费 boundary jumps/spans/traces，不读取全局 threshold 或 white ratio。

## 11. BEV Classification 与 hold

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `BEV_CLASSIFICATION.WHITE_CONFIDENCE_MIN` | `0.55` | debug/legacy classification | V9 active line/cross/CircleV2 不读取该值；仅保留给显式 debug/legacy/test 分类入口。不要用它调当前寻线。 |
| `BEV_CLASSIFICATION.UNKNOWN_CONFIDENCE_MIN` | `0.25` | debug/legacy classification | V9 active line/cross/CircleV2 不读取该值；仅保留给显式 debug/legacy/test 分类入口。不要用它调当前寻线。 |
| `BEV_CLASSIFICATION.HOLD_LAST_MAX_CYCLES` | `32` | reference continuity | 当前视觉 facts 不 usable 时最多 hold 上次路径的周期数。短暂丢点可增大；不想让旧路径影响控制就减小。hold 点必须显示 `reference.mode=hold_last`、`reference.source=hold`。 |

`BEV_CLASSIFICATION` 不再是 active sparse row authority。当前 active 视觉入口是 `BEV_BOUNDARY`；line/cross/CircleV2 调参优先看 boundary jumps/spans/traces 和相关 element evidence。

## 12. BEV Control Model

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `BEV_CONTROL_MODEL.LATERAL_OFFSET_TO_WHEEL_DELTA_GAIN` | `136.007362` | turn-output target | 旧增益 `150` 除以横向尺度，保持同一旧路径上的 lateral term 等效；尚未实车重调。 |
| `BEV_CONTROL_MODEL.HEADING_ERROR_TO_WHEEL_DELTA_GAIN` | `59.136130` | turn-output target | 旧增益 `50` 按小角度斜率变化 `forward_scale/lateral_scale` 换算；大航向误差下不是严格等价，需实车复核。 |
| `BEV_CONTROL_MODEL.CURVATURE_TO_WHEEL_DELTA_GAIN` | `23.141201` | turn-output target | 旧增益 `15` 按小斜率曲率变化 `forward_scale^2/lateral_scale` 换算；尚未实车重调。 |
| `BEV_CONTROL_MODEL.MIN_LEADING_REFERENCE_SAMPLES` | `3` | reference usability | 第一个连续真实 reference 点段的最小数量。近端丢线本身不使路径不可用，但真实连续点少于该值仍不可用。低于 3 时按 3 处理。 |
| `BEV_CONTROL_MODEL.TRACKING_FIT_MIN_SAMPLES` | `3` | reference tracking geometry | 二次拟合 `reference_tracking_geometry` 所需的最小 leading usable 样本数。合法范围 `[3, 24]`。 |

旧 `BEV_CONTROL_MODEL.LATERAL_ERROR_TO_WHEEL_DELTA_GAIN` 兼容别名已从 active runtime 移除。`reference_lateral_error` 只保留为固定权重的 legacy debug 对照事实，不再有 JSON 调参面。

## 13. BEV Element

Circle V2 架构见 `new/docs/visual-element-sparse-circle-v2.zh-CN.md`。运行时 circle 语义归 `CircleV2Scene` 所有；`RunVisualElementPipeline()` 只保留 cross / non-circle visual element evidence。旧 circle evidence 参数面已删除，不再作为运行时配置或媒体解释依据。

| 参数 | 当前 JSON 值 | 作用层 | 调参方法与证据 |
| --- | ---: | --- | --- |
| `BEV_ELEMENT.CROSS_EXIT_TAKEOVER_ENABLED` | `1` | visual element candidate inclusion | 默认开启。`element_evidence.cross_exit` 触发并构造 candidate 后可进入 visual-reference arbitration；最终仍必须通过 existing candidate validation、reference usability、tracking geometry、reference-control readiness 和 safety gate。 |
| `BEV_ELEMENT.CIRCLE_V2_ENABLED` | `1` | scene registry | CircleV2Scene 启动期组合开关。关闭时不注册 V2 场景；运行时热切换若存在，必须由组合层 reset scene memory，不属于 reducer 正常转移。 |
| `BEV_ELEMENT.CIRCLE_V2_EXIT_YAW_THRESHOLD_DEG` | `400` | CircleV2 B->C gate | InnerTrace 进入后的方向归一化累计 yaw 阈值。左/右符号由 CircleV2EventObserver 按锁存方向归一化，不使用 `abs(yaw_delta)`。 |
| `BEV_ELEMENT.CIRCLE_V2_EXIT_HOLD_FRAMES` | `120` | CircleV2 C hold | ExitTrace 输出保持帧数，同时承担 cooldown 职责。当前 `control_period_ms=5` 时约 `600ms`；小于 2 按参数解析失败处理。 |
| `BEV_ELEMENT.CIRCLE_V2_INNER_TRACE_STALL_TIMEOUT_MS` | `2000` | CircleV2 B stall fallback | InnerTrace 持续超过该时长且 directed yaw 仍没有明显累计时退回 Idle。合法值 `>=1`。 |
| `BEV_ELEMENT.CIRCLE_V2_INNER_TRACE_STALL_YAW_MIN_DEG` | `60` | CircleV2 B stall fallback | InnerTrace 超时兜底的“明显 yaw 积分”阈值。超时后 directed yaw 小于该值才退回 Idle。合法值 `0..720`。 |
| `BEV_ELEMENT.CIRCLE_V2_INNER_TRACE_PATH_OFFSET_M` | `0.0` | CircleV2 B path | InnerTrace 路径从内圆边线向道路内部偏移的距离。`0.0` 表示贴内圆边线；正值左环岛向右偏、右环岛向左偏。合法值 `0..2`。 |
| `BEV_ELEMENT.CIRCLE_V2_OPPOSITE_STRAIGHT_CONFIDENCE_MIN` | `0.7` | CircleV2 observer | CircleV2 Phase1 cue 和 Approach entry gate 使用“对侧直线”时的最低拟合置信度。`0.0` 等价旧行为；合法值 `0..1`。 |
| `BEV_ELEMENT.CIRCLE_V2_ENTRY_BOTTOM_MIN_ROW_COUNT` | `3` | CircleV2 Approach gate | Approach entry gate 在下部 ROI 内要求的最少证据行数。它不限制 ROI 内最多扫描多少行，不改变 Phase1 cue 的全局 trace 语义。合法值 `1..24`。 |
| `BEV_ELEMENT.CIRCLE_V2_ENTRY_BOTTOM_FORWARD_MIN_M` | `0.130440284` | CircleV2 Approach gate | 旧前向下限 `0.1` 按前向尺度换算。 |
| `BEV_ELEMENT.CIRCLE_V2_ENTRY_BOTTOM_FORWARD_MAX_M` | `0.456540995` | CircleV2 Approach gate | 旧前向上限 `0.35` 按前向尺度换算；完整 `[min,max]` 区间都会参与开口搜索。 |

`cross_exit` 第一版只用于 evidence/debug。不要为了让车“看起来过十字”而用它直接改 actuator、yaw、safety 或 hold。现场先在 no-motion capture 中确认 `element_evidence.cross_exit.{present,confidence,reason,candidate.*}` 与 raw/BEV 画面对齐。generic element 扩展记录统一在 `element_evidence.records[]`，旧消费者只读 `cross_exit` 即可。

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
`ML.ENABLED` defaults to `0`; in that state no ML
work is scheduled and zero-valued ROI calibration, odometry, and maneuver
fields are intentionally valid.

When `ML.ENABLED=1`, startup validation is fail-closed. The ROI search span,
forward/lateral grid steps, expected rectangle edges and their tolerances must be finite and
positive. YUV intervals must be ordered inside `[0,255]`; orientation must be
in `(0, pi/2]`; component count must be at least one; rectangularity and red
fill thresholds must be in `(0,1]`; score weights must be finite and
nonnegative with a positive sum. `MOTION_ODOMETRY.ENCODER_TICKS_TO_METER`,
`ML.MANEUVER.SPEED_TARGET`, and `ML.MANEUVER.EXIT_FORWARD_M` must be positive.
The duration and integration-gap limits must be at least 1 ms. Exit tolerances
and cooldown may be zero.

`ML.V9` accepts `MIN_MARGIN>=0`, `MAX_BEST_DISTANCE` in `[0,126]`, and
`CONFIRM_FRAMES>=1`. `ML.CLASS_MAPPING` is separate from acceptance. Each class
maps to one of `straight`, `left`, `right`, or `unmapped`; defaults are class 0
straight, class 1 left, and class 2 right. Duplicate mappings are permitted.
The calibrated default uses `CONFIRM_FRAMES=3`: the current live vehicle run
started with nine consecutive class-1 frames and therefore locks vehicle on
frame 3 before later isolated class-2 noise; the scene already stops inference
after this lock, so this is the intended confirmation contract rather than a
post-classification remap.

The current disabled-by-default ML ROI calibration consumes the corrected BEV
metric directly. Its marker truth is `EXPECTED_LONG_EDGE_M=0.120` and
`EXPECTED_SHORT_EDGE_M=0.050`; its search window was transformed with the same
projector scale about vehicle origin `(0,0)` to
`forward=[0.1304402844,0.2739245972]m` and
`lateral=+-0.1433745920m`. The old isotropic fictitious `GRID_STEP_M=0.003`
was removed because one value cannot represent two different axis scales;
the corrected contract is `GRID_FORWARD_STEP_M=0.0039132085` and
`GRID_LATERAL_STEP_M=0.0033086444`. No ML-only scale compensation exists.
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
