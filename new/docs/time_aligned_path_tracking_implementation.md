# 时间对齐的路径跟踪系统：互不知晓架构完整实现路径

> 代码基线：GitHub 快照 `454eaf6e6042cb24a7ea40492c1179a8bc11efef`。
> 目标：把当前 `reference_time_alignment` 从 **yaw-only / now-frame alignment** 升级为 **control-effective-time SE(2) alignment**。
> 设计约束：感知、状态估计、路径对齐、跟踪几何、控制器之间保持“互不知晓”；只有 `runtime::ControlLoop` 做编排。

---

## 0. 当前代码事实

### 0.1 `PerceptionResult` 已经有正确的“拍摄时刻路径”合同

当前 `PerceptionResult` 中同时保存：

- `capture_time_ms`
- `publish_time_ms`
- `reference_capture_time_ms`
- `reference_path`
- `reference_time_alignment`
- `reference_tracking_geometry`
- `reference_control`

也就是说，感知侧已经可以表达：

```text
Path(t_camera)
```

对应代码位置：

```cpp
uint64_t capture_time_ms = 0;
uint64_t publish_time_ms = 0;

uint64_t reference_capture_time_ms = 0;
BEVReferencePath reference_path{};
ReferenceTimeAlignmentFacts reference_time_alignment{};

ReferenceUsability reference_usability{};
ReferenceLateralErrorEstimate reference_lateral_error{};
ReferenceTrackingGeometry reference_tracking_geometry{};
ReferenceControlReadiness reference_control{};
```

来源：`new/code/port/perception_result.hpp`。

### 0.2 控制循环已经存在“control-time perception”插入点

当前 `ControlLoop::Tick()` 的顺序是：

```text
读 IMU / Encoder
→ 记录 motion_history
→ 复制 perception / motion_history / last_command
→ BuildControlTimePerception(...)
→ EvaluateControlGate(...)
→ MotionSupervisor
→ yaw_controller.ComputeTurnOutputTarget(...)
→ gyro feedback
→ wheel_target_mixer
→ wheel PID
→ actuator Apply
→ debug snapshot
```

关键代码：

```cpp
perception = BuildControlTimePerception(perception, motion_history, now_ms, params_);
```

而且 `BuildControlTimePerception()` 对齐后会重新计算：

```cpp
out.reference_usability = reference::EvaluateReferenceUsability(...);
out.reference_lateral_error = reference::ComputeReferenceLateralError(...);
out.reference_tracking_geometry = reference::ComputeReferenceTrackingGeometry(...);
out.reference_control = reference::EvaluateReferenceControlReadiness(...);
```

这说明你要实现的系统不应该重写感知，也不应该重写控制器；正确落点就是：

```text
BuildControlTimePerception()
```

### 0.3 当前 `reference_time_alignment` 还是 yaw-only

当前实现里：

```cpp
facts.delta_forward_m = 0.0;
facts.delta_yaw_rad = delta_yaw;
...
facts.reason = "aligned_yaw_only";
```

路径变换也只做：

```cpp
x = sample.forward_m - delta_forward_m;   // 旧实现中前向补偿恒为 0
y = sample.lateral_m;
R(-delta_yaw) * [x, y]
```

所以当前功能是：

```text
Path(t_camera) → Path(t_now)，但只补偿 yaw，不补偿 forward / lateral，也不预测到 control_effective_time。
```

### 0.4 控制器已经可以保持不变

`SteeringYawController` 只消费：

```cpp
const port::ReferenceTrackingGeometry& tracking_geometry
```

然后计算：

```cpp
lateral_term   = gain_lateral   * speed_scale * lateral_offset_m;
heading_term   = gain_heading   * speed_scale * heading_error_rad;
curvature_term = gain_curvature * speed_scale * curvature_m_inv;
turn_output    = lateral_term + heading_term + curvature_term;
```

因此最终系统应该让控制器仍然只看到：

```text
ReferenceTrackingGeometry
```

不要让它知道：

```text
延迟、IMU、编码器、路径时间戳、执行器响应模型
```

---

## 1. 最终架构

### 1.1 模块边界

最终链路：

```text
PerceptionPipeline
  输出 Path(t_camera)
  不知道控制延迟
  不知道 IMU / Encoder
  不知道控制器

MotionHistory
  只记录传感器事实
  不知道路径
  不知道控制器

CommandHistory
  只记录已请求/已施加的执行命令事实
  不知道路径
  不知道控制器内部算法

VehiclePoseDeltaEstimator
  输入 MotionHistory + CommandHistory + 时间窗口
  输出 VehiclePoseDelta(t_camera → t_effective)
  不知道 BEVReferencePath
  不知道 ReferenceTrackingGeometry
  不知道 yaw_controller

ReferenceTimeAlignment
  输入 BEVReferencePath + VehiclePoseDelta
  输出 aligned BEVReferencePath
  不知道 IMU / Encoder / CommandHistory
  不知道控制器

ReferenceTrackingGeometry
  输入 aligned BEVReferencePath
  输出 lateral_offset / heading / curvature
  不知道延迟补偿

SteeringYawController
  输入 ReferenceTrackingGeometry
  输出 turn_output_target
  不知道路径是拍摄时刻还是控制生效时刻
```

### 1.2 唯一知道全局链路的是 `runtime::ControlLoop`

`ControlLoop` 是编排层，允许知道所有模块：

```text
perception
motion_history
command_history
params
pose_delta_estimator
reference_time_alignment
reference_tracking_geometry
yaw_controller
wheel_mixer
actuator
```

但每个业务模块只知道自己的输入/输出合同。

---

## 2. 实施原则

### 2.1 不改感知语义

`PerceptionResult.reference_path` 在感知发布时仍表示：

```text
Path(t_camera)
```

不在相机线程里做时间补偿。

### 2.2 不改控制器语义

`SteeringYawController` 仍只处理：

```cpp
port::ReferenceTrackingGeometry
```

不新增 `delay_ms`、`pose_delta`、`motion_history` 参数。

### 2.3 对齐只发生在控制副本

控制循环里拿到 perception 后复制一份，然后只修改控制副本：

```cpp
port::PerceptionResult out = perception;
out.reference_path = aligned_path;
out.reference_tracking_geometry = ComputeReferenceTrackingGeometry(aligned_path, ...);
```

共享状态里的原始感知事实不被污染。

### 2.4 估计器输出的是“车体位姿增量”，不是路径

状态估计器的核心类型应该是：

```cpp
VehiclePoseDelta
```

不是：

```cpp
BEVReferencePath
```

这样状态估计器不需要知道视觉世界。

### 2.5 路径对齐器输入的是“路径 + 位姿增量”，不是传感器历史

`reference_time_alignment` 现在直接读 `MotionHistory`，这会让 reference 模块知道运动估计细节。最终应改成：

```cpp
AlignReferencePathToVehiclePoseDelta(path, pose_delta, params)
```

而不是：

```cpp
AlignReferencePathToControlTime(path, motion_history, params)
```

---

## 3. 新增 / 修改文件总览

```text
new/code/port/control_command_history_types.hpp          新增
new/code/port/vehicle_pose_delta_types.hpp               新增
new/code/estimation/vehicle_pose_delta_estimator.hpp     新增
new/code/estimation/vehicle_pose_delta_estimator.cpp     新增

new/code/port/bev_reference_types.hpp                    修改 ReferenceTimeAlignmentFacts
new/code/port/runtime_parameter_types.hpp                修改 ReferenceTimeAlignmentParameters
new/code/runtime/runtime_state.hpp                       增加 command_history
new/code/reference/reference_time_alignment.hpp          修改 API
new/code/reference/reference_time_alignment.cpp          改成 SE(2) path transform
new/code/runtime/loops/control_loop.cpp                  改 BuildControlTimePerception + command history 记录
new/code/observability/control_debug_snapshot.hpp        增加 debug 字段
new/code/platform/param_store.cpp                        增加参数读取和校验
new/config/default_params.json                           增加参数默认值
new/config/default_params.md                             同步文档

new/verification/tests/reference_time_alignment_test.cpp 新增 SE(2) 测试
new/verification/tests/vehicle_pose_delta_estimator_test.cpp 新增
new/verification/tests/run_reference_time_alignment_test.sh 修改编译源
new/verification/tests/run_vehicle_pose_delta_estimator_test.sh 新增
```

---

## 4. 第一步：新增命令历史类型

### 4.1 新文件

路径：

```text
new/code/port/control_command_history_types.hpp
```

完整代码：

```cpp
#ifndef LS2K_PORT_CONTROL_COMMAND_HISTORY_TYPES_HPP
#define LS2K_PORT_CONTROL_COMMAND_HISTORY_TYPES_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace ls2k::port {

/// 控制命令历史采样。
/// 只记录“已经请求/已经施加到执行器路径”的事实，不表达控制器算法语义。
struct ControlCommandHistorySample {
    uint64_t time_ms = 0;

    bool valid = false;
    bool actuator_applied = false;
    bool diagnostics_only = false;
    bool hold_disarmed = false;
    bool emergency_stop = false;

    int raw_turn_output = 0;
    int applied_turn_output = 0;

    double left_wheel_target = 0.0;
    double right_wheel_target = 0.0;

    int left_drive_pwm = 0;
    int right_drive_pwm = 0;
    int left_brushless_pwm = 0;
    int right_brushless_pwm = 0;
};

/// 固定容量控制命令历史 ring buffer。
struct ControlCommandHistory {
    static constexpr std::size_t kCapacity = 2048;

    void Push(const ControlCommandHistorySample& sample) {
        samples[next_index] = sample;
        next_index = (next_index + 1) % kCapacity;
        if (count < kCapacity) {
            ++count;
        }
    }

    void Clear() {
        samples = {};
        next_index = 0;
        count = 0;
    }

    const ControlCommandHistorySample& OldestOffset(std::size_t offset) const {
        const std::size_t src = (next_index + kCapacity - count + offset) % kCapacity;
        return samples[src];
    }

    const ControlCommandHistorySample& NewestOffset(std::size_t offset) const {
        const std::size_t src = (next_index + kCapacity - 1U - offset) % kCapacity;
        return samples[src];
    }

    bool LatestBeforeOrAt(uint64_t time_ms, ControlCommandHistorySample& out) const {
        for (std::size_t offset = 0; offset < count; ++offset) {
            const ControlCommandHistorySample& sample = NewestOffset(offset);
            if (sample.time_ms <= time_ms) {
                out = sample;
                return true;
            }
        }
        return false;
    }

    std::array<ControlCommandHistorySample, kCapacity> samples{};
    std::size_t next_index = 0;
    std::size_t count = 0;
};

}  // namespace ls2k::port

#endif  // LS2K_PORT_CONTROL_COMMAND_HISTORY_TYPES_HPP
```

### 4.2 为什么单独建 CommandHistory

不要扩展 `MotionHistorySample` 去塞命令字段。

当前 `MotionHistorySample` 注释已经明确：

```cpp
/// 控制侧运动历史采样，只记录传感器事实
```

所以它应该继续只保存：

```cpp
time_ms
imu_valid
gyro_z
encoder_valid
left_encoder_delta
right_encoder_delta
```

命令历史是另一类事实，应该独立保存。

---

## 5. 第二步：新增位姿增量类型

### 5.1 新文件

路径：

```text
new/code/port/vehicle_pose_delta_types.hpp
```

完整代码：

```cpp
#ifndef LS2K_PORT_VEHICLE_POSE_DELTA_TYPES_HPP
#define LS2K_PORT_VEHICLE_POSE_DELTA_TYPES_HPP

#include <cstddef>
#include <cstdint>
#include <string>

namespace ls2k::port {

/// 从 start_time_ms 到 end_time_ms 的车体位姿增量。
/// 坐标约定：forward 为车体前向，lateral 为车体右向，yaw 为逆时针/IMU 正方向，
/// 使用者必须通过测试固定符号约定。
struct VehiclePoseDelta {
    bool valid = false;
    std::string reason = "not_computed";

    uint64_t start_time_ms = 0;
    uint64_t now_time_ms = 0;
    uint64_t end_time_ms = 0;
    uint64_t measured_until_ms = 0;
    uint64_t predicted_ms = 0;

    double delta_forward_m = 0.0;
    double delta_lateral_m = 0.0;
    double delta_yaw_rad = 0.0;

    double measured_forward_mps = 0.0;
    double measured_yaw_rate_radps = 0.0;
    double predicted_forward_mps = 0.0;
    double predicted_yaw_rate_radps = 0.0;

    std::size_t integrated_motion_segments = 0;
    bool used_encoder_forward = false;
    bool used_imu_yaw = false;
    bool used_wheel_yaw = false;
    bool used_command_prediction = false;
};

}  // namespace ls2k::port

#endif  // LS2K_PORT_VEHICLE_POSE_DELTA_TYPES_HPP
```

---

## 6. 第三步：扩展参数类型

### 6.1 修改文件

路径：

```text
new/code/port/runtime_parameter_types.hpp
```

替换当前：

```cpp
struct ReferenceTimeAlignmentParameters {
    bool enabled = false;
    int max_age_ms = 120;
    int max_integration_gap_ms = 30;
    double max_delta_yaw_rad = 0.80;
    int min_aligned_samples = 3;
};
```

为：

```cpp
struct ReferenceTimeAlignmentParameters {
    bool enabled = false;                  ///< 是否启用控制侧 reference 时间对齐

    // 时间窗口
    int max_age_ms = 120;                  ///< reference 最大可用年龄
    int effective_delay_ms = 0;            ///< now -> control_effective 的估计延迟
    int future_prediction_max_ms = 80;     ///< 允许预测的最大未来时间

    // 历史积分安全边界
    int max_integration_gap_ms = 30;       ///< motion history 最大允许采样空洞
    int min_aligned_samples = 3;           ///< 对齐后最少前方样本数

    // 编码器 / 车体模型
    bool use_encoder_forward = false;      ///< 是否用编码器积分前进距离
    double encoder_ticks_to_meter = 0.0;   ///< 编码器 delta -> meter 的比例
    double wheel_track_m = 0.0;            ///< 左右轮距；用于轮速 yaw fallback

    // yaw 来源
    bool use_imu_yaw = true;               ///< 优先使用 IMU gyro_z 积分 yaw
    bool use_wheel_yaw_fallback = false;   ///< IMU 不可用时是否用左右轮差估 yaw

    // 未来预测
    bool future_prediction_enabled = false;        ///< 是否预测 now -> effective
    bool command_yaw_prediction_enabled = false;   ///< 是否用 applied_turn_output 预测 yaw_rate
    double turn_output_to_yaw_rate_gain = 0.0;     ///< applied_turn_output -> yaw_rate(rad/s)
    double actuator_yaw_tau_ms = 35.0;             ///< 差速执行一阶响应时间常数

    // fail-closed 门限
    double max_delta_forward_m = 0.60;      ///< 单次对齐最大前向位移
    double max_delta_lateral_m = 0.40;      ///< 单次对齐最大横向位移
    double max_delta_yaw_rad = 0.80;        ///< 单次对齐最大 yaw
};
```

### 6.2 参数默认策略

默认保持：

```cpp
enabled = false
use_encoder_forward = false
future_prediction_enabled = false
```

这样不改变当前运行行为。

启用顺序建议：

```text
阶段 A：enabled=1, effective_delay_ms=0, use_encoder_forward=0
阶段 B：enabled=1, effective_delay_ms=0, use_encoder_forward=1
阶段 C：effective_delay_ms=实测延迟, future_prediction_enabled=1
阶段 D：command_yaw_prediction_enabled=1
```

---

## 7. 第四步：扩展 ReferenceTimeAlignmentFacts

### 7.1 修改文件

路径：

```text
new/code/port/bev_reference_types.hpp
```

替换当前：

```cpp
struct ReferenceTimeAlignmentFacts {
    bool enabled = false;
    bool valid = false;
    std::string reason = "disabled";
    uint64_t age_ms = 0;
    uint64_t reference_capture_time_ms = 0;
    uint64_t control_time_ms = 0;
    double old_forward_offset_m = 0.0;
    double delta_yaw_rad = 0.0;
    std::size_t input_sample_count = 0;
    std::size_t aligned_sample_count = 0;
};
```

为最终版：

```cpp
struct ReferenceTimeAlignmentFacts {
    bool enabled = false;
    bool valid = false;
    std::string reason = "disabled";

    uint64_t age_ms = 0;
    uint64_t reference_capture_time_ms = 0;

    /// 当前控制循环时间。
    uint64_t control_time_ms = 0;

    /// 新字段：控制真正生效的目标时间。
    uint64_t control_effective_time_ms = 0;

    /// 历史积分实际覆盖到哪里。
    uint64_t measured_until_ms = 0;

    /// now -> effective 的预测时长。
    uint64_t predicted_ms = 0;

    double delta_forward_m = 0.0;
    double delta_lateral_m = 0.0;
    double delta_yaw_rad = 0.0;

    double measured_forward_mps = 0.0;
    double measured_yaw_rate_radps = 0.0;
    double predicted_forward_mps = 0.0;
    double predicted_yaw_rate_radps = 0.0;

    bool used_encoder_forward = false;
    bool used_imu_yaw = false;
    bool used_wheel_yaw = false;
    bool used_command_prediction = false;

    std::size_t input_sample_count = 0;
    std::size_t aligned_sample_count = 0;
};
```

---

## 8. 第五步：新增 VehiclePoseDeltaEstimator

### 8.1 新文件

路径：

```text
new/code/estimation/vehicle_pose_delta_estimator.hpp
```

代码：

```cpp
#ifndef LS2K_ESTIMATION_VEHICLE_POSE_DELTA_ESTIMATOR_HPP
#define LS2K_ESTIMATION_VEHICLE_POSE_DELTA_ESTIMATOR_HPP

#include <cstdint>

#include "port/control_command_history_types.hpp"
#include "port/motion_history_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "port/vehicle_pose_delta_types.hpp"

namespace ls2k::estimation {

port::VehiclePoseDelta EstimateVehiclePoseDelta(
    uint64_t start_time_ms,
    uint64_t now_time_ms,
    uint64_t end_time_ms,
    const port::MotionHistory& motion_history,
    const port::ControlCommandHistory& command_history,
    const port::ReferenceTimeAlignmentParameters& params);

}  // namespace ls2k::estimation

#endif  // LS2K_ESTIMATION_VEHICLE_POSE_DELTA_ESTIMATOR_HPP
```

### 8.2 新文件

路径：

```text
new/code/estimation/vehicle_pose_delta_estimator.cpp
```

代码：

```cpp
#include "estimation/vehicle_pose_delta_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ls2k::estimation {
namespace {

bool IsFinite(double value) {
    return std::isfinite(value);
}

void Fail(port::VehiclePoseDelta& out, const char* reason) {
    out.valid = false;
    out.reason = reason;
}

void IntegrateBodyStep(port::VehiclePoseDelta& out, double ds_m, double dyaw_rad) {
    const double theta_mid = out.delta_yaw_rad + 0.5 * dyaw_rad;
    out.delta_forward_m += std::cos(theta_mid) * ds_m;
    out.delta_lateral_m += std::sin(theta_mid) * ds_m;
    out.delta_yaw_rad += dyaw_rad;
}

bool LatestUsableCommand(const port::ControlCommandHistory& history,
                         uint64_t now_ms,
                         port::ControlCommandHistorySample& out) {
    for (std::size_t offset = 0; offset < history.count; ++offset) {
        const port::ControlCommandHistorySample& sample = history.NewestOffset(offset);
        if (sample.time_ms > now_ms) {
            continue;
        }
        if (!sample.valid || !sample.actuator_applied || sample.emergency_stop ||
            sample.diagnostics_only || sample.hold_disarmed) {
            continue;
        }
        out = sample;
        return true;
    }
    return false;
}

bool IntegrateMeasuredWindow(uint64_t start_ms,
                             uint64_t end_ms,
                             const port::MotionHistory& history,
                             const port::ReferenceTimeAlignmentParameters& params,
                             port::VehiclePoseDelta& out) {
    if (end_ms <= start_ms) {
        out.measured_until_ms = start_ms;
        return true;
    }
    if (history.count < 2) {
        Fail(out, "motion_history_unavailable");
        return false;
    }

    if (params.use_encoder_forward && params.encoder_ticks_to_meter <= 0.0) {
        Fail(out, "encoder_scale_unavailable");
        return false;
    }

    uint64_t covered_until_ms = start_ms;
    const uint64_t max_gap_ms = static_cast<uint64_t>(std::max(1, params.max_integration_gap_ms));

    double measured_s_m = 0.0;
    double measured_yaw_rad = 0.0;
    uint64_t measured_dt_ms = 0;

    for (std::size_t index = 1; index < history.count; ++index) {
        const port::MotionHistorySample& prev = history.OldestOffset(index - 1U);
        const port::MotionHistorySample& curr = history.OldestOffset(index);

        if (curr.time_ms <= covered_until_ms) {
            continue;
        }
        if (prev.time_ms > covered_until_ms || prev.time_ms >= end_ms) {
            break;
        }
        if (curr.time_ms <= prev.time_ms) {
            Fail(out, "motion_history_non_monotonic");
            return false;
        }

        const uint64_t full_gap_ms = curr.time_ms - prev.time_ms;
        if (full_gap_ms > max_gap_ms) {
            Fail(out, "motion_history_unavailable");
            return false;
        }

        const uint64_t segment_start_ms = std::max(covered_until_ms, prev.time_ms);
        const uint64_t segment_end_ms = std::min(end_ms, curr.time_ms);
        if (segment_end_ms <= segment_start_ms) {
            continue;
        }

        const double fraction =
            static_cast<double>(segment_end_ms - segment_start_ms) /
            static_cast<double>(full_gap_ms);
        const double dt_s = static_cast<double>(segment_end_ms - segment_start_ms) / 1000.0;

        double ds_m = 0.0;
        if (params.use_encoder_forward) {
            if (!curr.encoder_valid) {
                Fail(out, "encoder_history_unavailable");
                return false;
            }
            const double mean_ticks =
                0.5 * (static_cast<double>(curr.left_encoder_delta) +
                       static_cast<double>(curr.right_encoder_delta));
            ds_m = mean_ticks * params.encoder_ticks_to_meter * fraction;
            out.used_encoder_forward = true;
        }

        double dyaw_rad = 0.0;
        bool yaw_available = false;

        if (params.use_imu_yaw && prev.imu_valid && curr.imu_valid) {
            dyaw_rad = static_cast<double>(prev.gyro_z) * dt_s;
            yaw_available = true;
            out.used_imu_yaw = true;
        } else if (params.use_wheel_yaw_fallback &&
                   curr.encoder_valid &&
                   params.encoder_ticks_to_meter > 0.0 &&
                   params.wheel_track_m > 1.0e-6) {
            const double right_m =
                static_cast<double>(curr.right_encoder_delta) *
                params.encoder_ticks_to_meter *
                fraction;
            const double left_m =
                static_cast<double>(curr.left_encoder_delta) *
                params.encoder_ticks_to_meter *
                fraction;
            dyaw_rad = (right_m - left_m) / params.wheel_track_m;
            yaw_available = true;
            out.used_wheel_yaw = true;
        }

        if (!yaw_available) {
            Fail(out, "yaw_history_unavailable");
            return false;
        }

        IntegrateBodyStep(out, ds_m, dyaw_rad);

        measured_s_m += ds_m;
        measured_yaw_rad += dyaw_rad;
        measured_dt_ms += segment_end_ms - segment_start_ms;
        covered_until_ms = segment_end_ms;
        out.integrated_motion_segments += 1U;

        if (covered_until_ms >= end_ms) {
            out.measured_until_ms = end_ms;
            if (measured_dt_ms > 0) {
                const double measured_dt_s = static_cast<double>(measured_dt_ms) / 1000.0;
                out.measured_forward_mps = measured_s_m / measured_dt_s;
                out.measured_yaw_rate_radps = measured_yaw_rad / measured_dt_s;
            }
            return true;
        }
    }

    Fail(out, "motion_history_unavailable");
    return false;
}

double FirstOrderAverage(double current, double target, double dt_s, double tau_s) {
    if (dt_s <= 0.0) {
        return current;
    }
    if (tau_s <= 1.0e-6) {
        return target;
    }
    const double ratio = tau_s / dt_s;
    const double decay = std::exp(-dt_s / tau_s);
    return target + (current - target) * ratio * (1.0 - decay);
}

bool PredictFutureWindow(uint64_t now_ms,
                         uint64_t end_ms,
                         const port::ControlCommandHistory& command_history,
                         const port::ReferenceTimeAlignmentParameters& params,
                         port::VehiclePoseDelta& out) {
    if (end_ms <= now_ms) {
        return true;
    }

    const uint64_t future_ms = end_ms - now_ms;
    if (!params.future_prediction_enabled) {
        Fail(out, "future_prediction_disabled");
        return false;
    }
    if (future_ms > static_cast<uint64_t>(std::max(0, params.future_prediction_max_ms))) {
        Fail(out, "future_prediction_horizon_exceeded");
        return false;
    }

    const double dt_s = static_cast<double>(future_ms) / 1000.0;
    double future_v_mps = out.measured_forward_mps;
    double future_yaw_rate = out.measured_yaw_rate_radps;

    port::ControlCommandHistorySample command{};
    if (params.command_yaw_prediction_enabled &&
        LatestUsableCommand(command_history, now_ms, command)) {
        const double target_yaw_rate =
            static_cast<double>(command.applied_turn_output) *
            params.turn_output_to_yaw_rate_gain;
        future_yaw_rate = FirstOrderAverage(out.measured_yaw_rate_radps,
                                            target_yaw_rate,
                                            dt_s,
                                            params.actuator_yaw_tau_ms / 1000.0);
        out.used_command_prediction = true;
    }

    out.predicted_ms = future_ms;
    out.predicted_forward_mps = future_v_mps;
    out.predicted_yaw_rate_radps = future_yaw_rate;

    const double ds_m = future_v_mps * dt_s;
    const double dyaw_rad = future_yaw_rate * dt_s;
    IntegrateBodyStep(out, ds_m, dyaw_rad);
    return true;
}

}  // namespace

port::VehiclePoseDelta EstimateVehiclePoseDelta(
    uint64_t start_time_ms,
    uint64_t now_time_ms,
    uint64_t end_time_ms,
    const port::MotionHistory& motion_history,
    const port::ControlCommandHistory& command_history,
    const port::ReferenceTimeAlignmentParameters& params) {
    port::VehiclePoseDelta out{};
    out.start_time_ms = start_time_ms;
    out.now_time_ms = now_time_ms;
    out.end_time_ms = end_time_ms;
    out.measured_until_ms = start_time_ms;

    if (start_time_ms == 0 || now_time_ms < start_time_ms || end_time_ms < start_time_ms) {
        Fail(out, "invalid_pose_delta_time");
        return out;
    }

    const uint64_t measured_end_ms = std::min(now_time_ms, end_time_ms);
    if (!IntegrateMeasuredWindow(start_time_ms, measured_end_ms, motion_history, params, out)) {
        return out;
    }

    if (!PredictFutureWindow(now_time_ms, end_time_ms, command_history, params, out)) {
        return out;
    }

    if (!IsFinite(out.delta_forward_m) ||
        !IsFinite(out.delta_lateral_m) ||
        !IsFinite(out.delta_yaw_rad)) {
        Fail(out, "pose_delta_nonfinite");
        return out;
    }

    if (std::fabs(out.delta_forward_m) > params.max_delta_forward_m) {
        Fail(out, "delta_forward_exceeded");
        return out;
    }
    if (std::fabs(out.delta_lateral_m) > params.max_delta_lateral_m) {
        Fail(out, "delta_lateral_exceeded");
        return out;
    }
    if (std::fabs(out.delta_yaw_rad) > params.max_delta_yaw_rad) {
        Fail(out, "delta_yaw_exceeded");
        return out;
    }

    out.valid = true;
    out.reason = out.predicted_ms > 0 ? "estimated_with_prediction" : "estimated_measured_only";
    return out;
}

}  // namespace ls2k::estimation
```

---

## 9. 第六步：改造 ReferenceTimeAlignment API

### 9.1 修改头文件

路径：

```text
new/code/reference/reference_time_alignment.hpp
```

当前头文件直接 include：

```cpp
#include "port/motion_history_types.hpp"
```

最终应移除它，改为 include：

```cpp
#include "port/vehicle_pose_delta_types.hpp"
```

推荐头文件：

```cpp
#ifndef LS2K_REFERENCE_REFERENCE_TIME_ALIGNMENT_HPP
#define LS2K_REFERENCE_REFERENCE_TIME_ALIGNMENT_HPP

#include <cstdint>

#include "port/bev_reference_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "port/vehicle_pose_delta_types.hpp"

namespace ls2k::reference {

struct ReferenceTimeAlignmentResult {
    port::BEVReferencePath reference_path{};
    port::ReferenceTimeAlignmentFacts facts{};
};

/// 将 capture-time reference path 变换到 control-effective-time 车体坐标系。
/// 注意：本模块不读取 IMU、encoder、command history；只消费 VehiclePoseDelta。
ReferenceTimeAlignmentResult AlignReferencePathToVehiclePoseDelta(
    const port::BEVReferencePath& reference_path,
    uint64_t reference_capture_time_ms,
    uint64_t control_time_ms,
    uint64_t control_effective_time_ms,
    const port::VehiclePoseDelta& pose_delta,
    const port::RuntimeParameters& params);

}  // namespace ls2k::reference

#endif  // LS2K_REFERENCE_REFERENCE_TIME_ALIGNMENT_HPP
```

### 9.2 修改实现文件

路径：

```text
new/code/reference/reference_time_alignment.cpp
```

核心替换点：

#### 9.2.1 删除 `IntegrateYawOnly`

这个函数应该被移动到估计器思路里；`reference_time_alignment` 不再知道 motion history。

删除：

```cpp
bool IntegrateYawOnly(...)
```

#### 9.2.2 替换 `TransformReferencePrefix`

当前：

```cpp
port::BEVReferencePath TransformReferencePrefix(
    const port::BEVReferencePath& input,
    std::size_t prefix_length,
    double delta_forward_m,
    double delta_yaw_rad,
    std::size_t& aligned_count)
```

替换为：

```cpp
port::BEVReferencePath TransformReferencePrefixSE2(
    const port::BEVReferencePath& input,
    std::size_t prefix_length,
    double delta_forward_m,
    double delta_lateral_m,
    double delta_yaw_rad,
    std::size_t& aligned_count) {
    port::BEVReferencePath output{};
    output.mode = input.mode;

    const double c = std::cos(-delta_yaw_rad);
    const double s = std::sin(-delta_yaw_rad);

    std::size_t out_index = 0;
    const std::size_t sample_count = std::min(prefix_length, input.sampled_path.size());

    for (std::size_t index = 0; index < sample_count; ++index) {
        const port::BEVPathSample& sample = input.sampled_path[index];

        const double x_rel =
            static_cast<double>(sample.point.forward_m) - delta_forward_m;
        const double y_rel =
            static_cast<double>(sample.point.lateral_m) - delta_lateral_m;

        const double forward = c * x_rel - s * y_rel;
        const double lateral = s * x_rel + c * y_rel;

        if (!std::isfinite(forward) || !std::isfinite(lateral) || forward <= 0.0) {
            continue;
        }

        if (out_index >= output.sampled_path.size()) {
            break;
        }

        port::BEVPathSample& out = output.sampled_path[out_index++];
        out = sample;
        out.point.forward_m = static_cast<float>(forward);
        out.point.lateral_m = static_cast<float>(lateral);
    }

    aligned_count = out_index;
    if (aligned_count == 0U) {
        output.mode = port::ReferenceMode::kNone;
    }
    return output;
}
```

#### 9.2.3 新主函数

```cpp
ReferenceTimeAlignmentResult AlignReferencePathToVehiclePoseDelta(
    const port::BEVReferencePath& reference_path,
    uint64_t reference_capture_time_ms,
    uint64_t control_time_ms,
    uint64_t control_effective_time_ms,
    const port::VehiclePoseDelta& pose_delta,
    const port::RuntimeParameters& params) {
    LS2K_PERF_SCOPE(port::PerfStage::kReferenceTimeAlignment);

    ReferenceTimeAlignmentResult result{};
    result.reference_path = reference_path;

    port::ReferenceTimeAlignmentFacts& facts = result.facts;
    facts.enabled = params.reference_time_alignment.enabled;
    facts.reference_capture_time_ms = reference_capture_time_ms;
    facts.control_time_ms = control_time_ms;
    facts.control_effective_time_ms = control_effective_time_ms;
    facts.input_sample_count = CountPresentSamples(reference_path);

    if (!params.reference_time_alignment.enabled) {
        facts.valid = true;
        facts.reason = "disabled";
        facts.aligned_sample_count = facts.input_sample_count;
        return result;
    }

    if (reference_capture_time_ms == 0 ||
        control_time_ms < reference_capture_time_ms ||
        control_effective_time_ms < reference_capture_time_ms) {
        facts.reason = "invalid_reference_time";
        result.reference_path = {};
        return result;
    }

    facts.age_ms = control_effective_time_ms - reference_capture_time_ms;
    if (facts.age_ms >
        static_cast<uint64_t>(std::max(1, params.reference_time_alignment.max_age_ms))) {
        facts.reason = "age_exceeded";
        result.reference_path = {};
        return result;
    }

    if (!pose_delta.valid) {
        facts.reason = "pose_delta_" + pose_delta.reason;
        result.reference_path = {};
        return result;
    }

    facts.measured_until_ms = pose_delta.measured_until_ms;
    facts.predicted_ms = pose_delta.predicted_ms;

    facts.delta_forward_m = pose_delta.delta_forward_m;
    facts.delta_lateral_m = pose_delta.delta_lateral_m;
    facts.delta_yaw_rad = pose_delta.delta_yaw_rad;

    facts.measured_forward_mps = pose_delta.measured_forward_mps;
    facts.measured_yaw_rate_radps = pose_delta.measured_yaw_rate_radps;
    facts.predicted_forward_mps = pose_delta.predicted_forward_mps;
    facts.predicted_yaw_rate_radps = pose_delta.predicted_yaw_rate_radps;

    facts.used_encoder_forward = pose_delta.used_encoder_forward;
    facts.used_imu_yaw = pose_delta.used_imu_yaw;
    facts.used_wheel_yaw = pose_delta.used_wheel_yaw;
    facts.used_command_prediction = pose_delta.used_command_prediction;

    if (std::fabs(facts.delta_forward_m) > params.reference_time_alignment.max_delta_forward_m) {
        facts.reason = "delta_forward_exceeded";
        result.reference_path = {};
        return result;
    }
    if (std::fabs(facts.delta_lateral_m) > params.reference_time_alignment.max_delta_lateral_m) {
        facts.reason = "delta_lateral_exceeded";
        result.reference_path = {};
        return result;
    }
    if (std::fabs(facts.delta_yaw_rad) > params.reference_time_alignment.max_delta_yaw_rad) {
        facts.reason = "delta_yaw_exceeded";
        result.reference_path = {};
        return result;
    }

    const std::size_t observed_prefix_length =
        LeadingObservedReferencePrefixLength(reference_path);

    result.reference_path = TransformReferencePrefixSE2(reference_path,
                                                        observed_prefix_length,
                                                        facts.delta_forward_m,
                                                        facts.delta_lateral_m,
                                                        facts.delta_yaw_rad,
                                                        facts.aligned_sample_count);

    if (facts.aligned_sample_count <
        static_cast<std::size_t>(std::max(1, params.reference_time_alignment.min_aligned_samples))) {
        facts.reason = "aligned_samples_insufficient";
        result.reference_path = {};
        return result;
    }

    facts.valid = true;
    facts.reason = facts.predicted_ms > 0 ? "aligned_effective_se2" : "aligned_measured_se2";
    return result;
}
```

---

## 10. 第七步：改造 RuntimeState

### 10.1 修改 include

路径：

```text
new/code/runtime/runtime_state.hpp
```

新增 include：

```cpp
#include "port/control_command_history_types.hpp"
```

### 10.2 修改结构体

在当前字段：

```cpp
port::ActuatorCommand last_command{};
...
port::MotionHistory motion_history{};
```

附近新增：

```cpp
port::ControlCommandHistory command_history{};  ///< control tick 命令历史
```

推荐位置：

```cpp
port::ActuatorCommand last_command{};
safety::ControlCycleObservation control_observation{};
observability::ControlDebugSnapshot control_debug_snapshot{};
port::LowVoltageSample low_voltage_last_sample{};
port::MotionHistory motion_history{};
port::ControlCommandHistory command_history{};
```

---

## 11. 第八步：改造 ControlLoop 编排

### 11.1 修改 include

路径：

```text
new/code/runtime/loops/control_loop.cpp
```

新增：

```cpp
#include "estimation/vehicle_pose_delta_estimator.hpp"
```

保留：

```cpp
#include "reference/reference_time_alignment.hpp"
```

### 11.2 修改 `BuildControlTimePerception` 签名

当前：

```cpp
port::PerceptionResult BuildControlTimePerception(
    const port::PerceptionResult& perception,
    const port::MotionHistory& motion_history,
    uint64_t now_ms,
    const port::RuntimeParameters& params)
```

替换为：

```cpp
port::PerceptionResult BuildControlTimePerception(
    const port::PerceptionResult& perception,
    const port::MotionHistory& motion_history,
    const port::ControlCommandHistory& command_history,
    uint64_t now_ms,
    const port::RuntimeParameters& params)
```

### 11.3 修改 `BuildControlTimePerception` 内容

替换内部 alignment 调用。

当前：

```cpp
const reference::ReferenceTimeAlignmentResult alignment =
    reference::AlignReferencePathToControlTime(perception.reference_path,
                                               perception.reference_capture_time_ms,
                                               now_ms,
                                               motion_history,
                                               params);
```

替换为：

```cpp
const uint64_t effective_delay_ms =
    static_cast<uint64_t>(std::max(0, params.reference_time_alignment.effective_delay_ms));
const uint64_t control_effective_time_ms = now_ms + effective_delay_ms;

const port::VehiclePoseDelta pose_delta =
    estimation::EstimateVehiclePoseDelta(perception.reference_capture_time_ms,
                                         now_ms,
                                         control_effective_time_ms,
                                         motion_history,
                                         command_history,
                                         params.reference_time_alignment);

const reference::ReferenceTimeAlignmentResult alignment =
    reference::AlignReferencePathToVehiclePoseDelta(perception.reference_path,
                                                   perception.reference_capture_time_ms,
                                                   now_ms,
                                                   control_effective_time_ms,
                                                   pose_delta,
                                                   params);
```

### 11.4 修改 early return 的 facts

当前 disabled / unavailable 分支只填：

```cpp
enabled
valid
reason
```

建议改成：

```cpp
out.reference_time_alignment.enabled = params.reference_time_alignment.enabled;
out.reference_time_alignment.valid = !params.reference_time_alignment.enabled;
out.reference_time_alignment.reason =
    params.reference_time_alignment.enabled ? "perception_unavailable" : "disabled";
out.reference_time_alignment.reference_capture_time_ms = perception.reference_capture_time_ms;
out.reference_time_alignment.control_time_ms = now_ms;
out.reference_time_alignment.control_effective_time_ms =
    now_ms + static_cast<uint64_t>(std::max(0, params.reference_time_alignment.effective_delay_ms));
```

### 11.5 修改 Tick 中共享状态读取

当前局部变量：

```cpp
port::MotionHistory motion_history{};
```

新增：

```cpp
port::ControlCommandHistory command_history{};
```

锁内复制：

```cpp
motion_history = state_.motion_history;
command_history = state_.command_history;
```

调用处替换：

```cpp
perception = BuildControlTimePerception(perception,
                                         motion_history,
                                         command_history,
                                         now_ms,
                                         params_);
```

### 11.6 记录 command_history

在 actuator apply、observation、debug snapshot 之后，最后锁内写回状态之前，构建 sample：

```cpp
port::ControlCommandHistorySample command_history_sample{};
command_history_sample.time_ms = now_ms;
command_history_sample.valid = true;
command_history_sample.actuator_applied =
    apply_ok && !diagnostics_only_actuator && !hold_disarmed && !command.emergency_stop;
command_history_sample.diagnostics_only = diagnostics_only_actuator;
command_history_sample.hold_disarmed = hold_disarmed;
command_history_sample.emergency_stop = command.emergency_stop;
command_history_sample.raw_turn_output = raw_turn_output;
command_history_sample.applied_turn_output = applied_turn_output;
command_history_sample.left_wheel_target = wheel_targets.left;
command_history_sample.right_wheel_target = wheel_targets.right;
command_history_sample.left_drive_pwm = command.left_drive_pwm;
command_history_sample.right_drive_pwm = command.right_drive_pwm;
command_history_sample.left_brushless_pwm = command.left_brushless_pwm;
command_history_sample.right_brushless_pwm = command.right_brushless_pwm;
```

然后在最后的锁内：

```cpp
state_.command_history.Push(command_history_sample);
```

完整插入位置建议：

```cpp
{
    std::lock_guard<std::mutex> lock(state_.shared_mutex);
    state_.motion_state = final_motion.state;
    ...
    state_.last_command = ...;
    state_.control_observation = observation;
    state_.control_debug_snapshot = debug_snapshot;
    state_.actuators_armed = observation.actuators_armed;
    state_.command_history.Push(command_history_sample);
}
```

### 11.7 reset 时清空 command_history

在这些函数里加入：

```cpp
state_.command_history.Clear();
```

位置：

```cpp
ControlLoop::Start()
ControlLoop::ResetDisarmedControlState()
ControlLoop::LatchTimerFailureState()
ResetControllerState(...)  // 如果传入 state，则可以一并清
```

其中 `ResetControllerState(...)` 当前已经有 `RuntimeState& state`，可以加入：

```cpp
state.command_history.Clear();
```

---

## 12. 第九步：扩展 Debug Snapshot

### 12.1 修改类型

路径：

```text
new/code/observability/control_debug_snapshot.hpp
```

当前 `ReferenceTimeAlignmentDebugView`：

```cpp
struct ReferenceTimeAlignmentDebugView {
    bool enabled = false;
    bool valid = false;
    std::string reason = "disabled";
    std::uint64_t age_ms = 0;
    std::uint64_t reference_capture_time_ms = 0;
    std::uint64_t control_time_ms = 0;
    double old_forward_offset_m = 0.0;
    double delta_yaw_rad = 0.0;
    std::size_t input_sample_count = 0;
    std::size_t aligned_sample_count = 0;
};
```

扩展为：

```cpp
struct ReferenceTimeAlignmentDebugView {
    bool enabled = false;
    bool valid = false;
    std::string reason = "disabled";

    std::uint64_t age_ms = 0;
    std::uint64_t reference_capture_time_ms = 0;
    std::uint64_t control_time_ms = 0;
    std::uint64_t control_effective_time_ms = 0;
    std::uint64_t measured_until_ms = 0;
    std::uint64_t predicted_ms = 0;

    double delta_forward_m = 0.0;
    double delta_lateral_m = 0.0;
    double delta_yaw_rad = 0.0;

    double measured_forward_mps = 0.0;
    double measured_yaw_rate_radps = 0.0;
    double predicted_forward_mps = 0.0;
    double predicted_yaw_rate_radps = 0.0;

    bool used_encoder_forward = false;
    bool used_imu_yaw = false;
    bool used_wheel_yaw = false;
    bool used_command_prediction = false;

    std::size_t input_sample_count = 0;
    std::size_t aligned_sample_count = 0;
};
```

### 12.2 修改 `BuildControlDebugSnapshot`

路径：

```text
new/code/runtime/loops/control_loop.cpp
```

当前已经复制：

```cpp
debug_snapshot.steering.reference_time_alignment.enabled = ...
debug_snapshot.steering.reference_time_alignment.valid = ...
debug_snapshot.steering.reference_time_alignment.reason = ...
debug_snapshot.steering.reference_time_alignment.age_ms = ...
debug_snapshot.steering.reference_time_alignment.reference_capture_time_ms = ...
debug_snapshot.steering.reference_time_alignment.control_time_ms = ...
debug_snapshot.steering.reference_time_alignment.delta_forward_m = ...
debug_snapshot.steering.reference_time_alignment.delta_yaw_rad = ...
debug_snapshot.steering.reference_time_alignment.input_sample_count = ...
debug_snapshot.steering.reference_time_alignment.aligned_sample_count = ...
```

在这里补充：

```cpp
debug_snapshot.steering.reference_time_alignment.control_effective_time_ms =
    perception.reference_time_alignment.control_effective_time_ms;
debug_snapshot.steering.reference_time_alignment.measured_until_ms =
    perception.reference_time_alignment.measured_until_ms;
debug_snapshot.steering.reference_time_alignment.predicted_ms =
    perception.reference_time_alignment.predicted_ms;
debug_snapshot.steering.reference_time_alignment.delta_forward_m =
    perception.reference_time_alignment.delta_forward_m;
debug_snapshot.steering.reference_time_alignment.delta_lateral_m =
    perception.reference_time_alignment.delta_lateral_m;
debug_snapshot.steering.reference_time_alignment.measured_forward_mps =
    perception.reference_time_alignment.measured_forward_mps;
debug_snapshot.steering.reference_time_alignment.measured_yaw_rate_radps =
    perception.reference_time_alignment.measured_yaw_rate_radps;
debug_snapshot.steering.reference_time_alignment.predicted_forward_mps =
    perception.reference_time_alignment.predicted_forward_mps;
debug_snapshot.steering.reference_time_alignment.predicted_yaw_rate_radps =
    perception.reference_time_alignment.predicted_yaw_rate_radps;
debug_snapshot.steering.reference_time_alignment.used_encoder_forward =
    perception.reference_time_alignment.used_encoder_forward;
debug_snapshot.steering.reference_time_alignment.used_imu_yaw =
    perception.reference_time_alignment.used_imu_yaw;
debug_snapshot.steering.reference_time_alignment.used_wheel_yaw =
    perception.reference_time_alignment.used_wheel_yaw;
debug_snapshot.steering.reference_time_alignment.used_command_prediction =
    perception.reference_time_alignment.used_command_prediction;
```

---

## 13. 第十步：扩展参数加载

### 13.1 修改校验

路径：

```text
new/code/platform/param_store.cpp
```

当前：

```cpp
bool ValidateReferenceTimeAlignment(const port::ReferenceTimeAlignmentParameters& params) {
    return params.max_age_ms >= 1 &&
           params.max_integration_gap_ms >= 1 &&
           IsFiniteInRange(params.max_delta_yaw_rad, 0.0, 6.28319) &&
           params.min_aligned_samples >= 1;
}
```

替换为：

```cpp
bool ValidateReferenceTimeAlignment(const port::ReferenceTimeAlignmentParameters& params) {
    return params.max_age_ms >= 1 &&
           params.effective_delay_ms >= 0 &&
           params.future_prediction_max_ms >= 0 &&
           params.max_integration_gap_ms >= 1 &&
           params.min_aligned_samples >= 1 &&
           IsFiniteInRange(params.encoder_ticks_to_meter, 0.0, 1.0) &&
           IsFiniteInRange(params.wheel_track_m, 0.0, 2.0) &&
           IsFiniteInRange(params.turn_output_to_yaw_rate_gain, -100.0, 100.0) &&
           IsFiniteInRange(params.actuator_yaw_tau_ms, 0.0, 1000.0) &&
           IsFiniteInRange(params.max_delta_forward_m, 0.0, 2.0) &&
           IsFiniteInRange(params.max_delta_lateral_m, 0.0, 2.0) &&
           IsFiniteInRange(params.max_delta_yaw_rad, 0.0, 6.28319);
}
```

### 13.2 修改读取

在当前读取 `REFERENCE_TIME_ALIGNMENT` 的位置，已有：

```cpp
ReadOptionalNestedBool(root, "REFERENCE_TIME_ALIGNMENT", "ENABLED", ...);
ReadOptionalNestedInt(root, "REFERENCE_TIME_ALIGNMENT", "MAX_AGE_MS", ...);
ReadOptionalNestedInt(root, "REFERENCE_TIME_ALIGNMENT", "MAX_INTEGRATION_GAP_MS", ...);
ReadOptionalNestedNumber(root, "REFERENCE_TIME_ALIGNMENT", "MAX_DELTA_YAW_RAD", ...);
ReadOptionalNestedInt(root, "REFERENCE_TIME_ALIGNMENT", "MIN_ALIGNED_SAMPLES", ...);
```

补充：

```cpp
ReadOptionalNestedInt(root,
                      "REFERENCE_TIME_ALIGNMENT",
                      "EFFECTIVE_DELAY_MS",
                      parsed.reference_time_alignment.effective_delay_ms,
                      optional_malformed);
ReadOptionalNestedInt(root,
                      "REFERENCE_TIME_ALIGNMENT",
                      "FUTURE_PREDICTION_MAX_MS",
                      parsed.reference_time_alignment.future_prediction_max_ms,
                      optional_malformed);
ReadOptionalNestedBool(root,
                       "REFERENCE_TIME_ALIGNMENT",
                       "USE_ENCODER_FORWARD",
                       parsed.reference_time_alignment.use_encoder_forward,
                       optional_malformed);
ReadOptionalNestedNumber(root,
                         "REFERENCE_TIME_ALIGNMENT",
                         "ENCODER_TICKS_TO_METER",
                         parsed.reference_time_alignment.encoder_ticks_to_meter,
                         optional_malformed);
ReadOptionalNestedNumber(root,
                         "REFERENCE_TIME_ALIGNMENT",
                         "WHEEL_TRACK_M",
                         parsed.reference_time_alignment.wheel_track_m,
                         optional_malformed);
ReadOptionalNestedBool(root,
                       "REFERENCE_TIME_ALIGNMENT",
                       "USE_IMU_YAW",
                       parsed.reference_time_alignment.use_imu_yaw,
                       optional_malformed);
ReadOptionalNestedBool(root,
                       "REFERENCE_TIME_ALIGNMENT",
                       "USE_WHEEL_YAW_FALLBACK",
                       parsed.reference_time_alignment.use_wheel_yaw_fallback,
                       optional_malformed);
ReadOptionalNestedBool(root,
                       "REFERENCE_TIME_ALIGNMENT",
                       "FUTURE_PREDICTION_ENABLED",
                       parsed.reference_time_alignment.future_prediction_enabled,
                       optional_malformed);
ReadOptionalNestedBool(root,
                       "REFERENCE_TIME_ALIGNMENT",
                       "COMMAND_YAW_PREDICTION_ENABLED",
                       parsed.reference_time_alignment.command_yaw_prediction_enabled,
                       optional_malformed);
ReadOptionalNestedNumber(root,
                         "REFERENCE_TIME_ALIGNMENT",
                         "TURN_OUTPUT_TO_YAW_RATE_GAIN",
                         parsed.reference_time_alignment.turn_output_to_yaw_rate_gain,
                         optional_malformed);
ReadOptionalNestedNumber(root,
                         "REFERENCE_TIME_ALIGNMENT",
                         "ACTUATOR_YAW_TAU_MS",
                         parsed.reference_time_alignment.actuator_yaw_tau_ms,
                         optional_malformed);
ReadOptionalNestedNumber(root,
                         "REFERENCE_TIME_ALIGNMENT",
                         "MAX_DELTA_FORWARD_M",
                         parsed.reference_time_alignment.max_delta_forward_m,
                         optional_malformed);
ReadOptionalNestedNumber(root,
                         "REFERENCE_TIME_ALIGNMENT",
                         "MAX_DELTA_LATERAL_M",
                         parsed.reference_time_alignment.max_delta_lateral_m,
                         optional_malformed);
```

---

## 14. 第十一步：扩展默认配置

### 14.1 修改 JSON

路径：

```text
new/config/default_params.json
```

当前：

```json
"REFERENCE_TIME_ALIGNMENT": {
  "ENABLED": 0,
  "MAX_AGE_MS": 120,
  "MAX_INTEGRATION_GAP_MS": 30,
  "MAX_DELTA_YAW_RAD": 0.8,
  "MIN_ALIGNED_SAMPLES": 3
}
```

替换为：

```json
"REFERENCE_TIME_ALIGNMENT": {
  "ENABLED": 0,

  "MAX_AGE_MS": 120,
  "EFFECTIVE_DELAY_MS": 0,
  "FUTURE_PREDICTION_MAX_MS": 80,

  "MAX_INTEGRATION_GAP_MS": 30,
  "MIN_ALIGNED_SAMPLES": 3,

  "USE_ENCODER_FORWARD": 0,
  "ENCODER_TICKS_TO_METER": 0.0,
  "WHEEL_TRACK_M": 0.0,

  "USE_IMU_YAW": 1,
  "USE_WHEEL_YAW_FALLBACK": 0,

  "FUTURE_PREDICTION_ENABLED": 0,
  "COMMAND_YAW_PREDICTION_ENABLED": 0,
  "TURN_OUTPUT_TO_YAW_RATE_GAIN": 0.0,
  "ACTUATOR_YAW_TAU_MS": 35.0,

  "MAX_DELTA_FORWARD_M": 0.6,
  "MAX_DELTA_LATERAL_M": 0.4,
  "MAX_DELTA_YAW_RAD": 0.8
}
```

### 14.2 为什么默认关掉 encoder forward

因为当前代码里编码器 delta 的物理尺度没有在这些文件里显式表达。直接默认打开会让系统依赖未校准比例。

上线时应先测：

```text
encoder_ticks_to_meter = 实际前进距离 / 左右 encoder delta 均值
```

---

## 15. 第十二步：测试策略

### 15.1 新增 VehiclePoseDeltaEstimator 测试

路径：

```text
new/verification/tests/vehicle_pose_delta_estimator_test.cpp
```

建议测试：

```cpp
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "estimation/vehicle_pose_delta_estimator.hpp"

namespace {

void Expect(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void ExpectNear(double actual, double expected, double tolerance, const char* message) {
    if (std::fabs(actual - expected) > tolerance) {
        throw std::runtime_error(message);
    }
}

ls2k::port::ReferenceTimeAlignmentParameters Params() {
    ls2k::port::ReferenceTimeAlignmentParameters params{};
    params.enabled = true;
    params.max_age_ms = 120;
    params.max_integration_gap_ms = 30;
    params.use_encoder_forward = true;
    params.encoder_ticks_to_meter = 0.001;
    params.use_imu_yaw = true;
    params.max_delta_forward_m = 1.0;
    params.max_delta_lateral_m = 1.0;
    params.max_delta_yaw_rad = 1.0;
    params.future_prediction_enabled = false;
    return params;
}

ls2k::port::MotionHistory MakeHistory() {
    ls2k::port::MotionHistory history{};
    history.Push({100, true, 0.0F, true, 0, 0});
    history.Push({110, true, 0.0F, true, 10, 10});
    history.Push({120, true, 0.0F, true, 10, 10});
    history.Push({130, true, 0.0F, true, 10, 10});
    return history;
}

void TestEncoderForward() {
    const auto result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        130,
        MakeHistory(),
        ls2k::port::ControlCommandHistory{},
        Params());

    Expect(result.valid, "encoder forward delta should be valid");
    ExpectNear(result.delta_forward_m, 0.03, 1.0e-6, "forward delta mismatch");
    ExpectNear(result.delta_lateral_m, 0.0, 1.0e-6, "lateral delta mismatch");
    ExpectNear(result.delta_yaw_rad, 0.0, 1.0e-6, "yaw delta mismatch");
}

void TestImuYaw() {
    auto history = MakeHistory();
    for (std::size_t i = 0; i < history.count; ++i) {
        history.samples[i].gyro_z = 1.0F;
    }

    const auto result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        130,
        history,
        ls2k::port::ControlCommandHistory{},
        Params());

    Expect(result.valid, "imu yaw delta should be valid");
    ExpectNear(result.delta_yaw_rad, 0.03, 1.0e-6, "yaw delta mismatch");
}

void TestFuturePredictionDisabledFailsWhenNeeded() {
    auto params = Params();
    params.future_prediction_enabled = false;

    const auto result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        150,
        MakeHistory(),
        ls2k::port::ControlCommandHistory{},
        params);

    Expect(!result.valid, "future prediction disabled should fail for future end time");
    Expect(result.reason == "future_prediction_disabled", "future prediction disabled reason mismatch");
}

void TestFutureConstantVelocity() {
    auto params = Params();
    params.future_prediction_enabled = true;
    params.future_prediction_max_ms = 80;

    const auto result = ls2k::estimation::EstimateVehiclePoseDelta(
        100,
        130,
        150,
        MakeHistory(),
        ls2k::port::ControlCommandHistory{},
        params);

    Expect(result.valid, "future constant velocity prediction should be valid");
    // measured 0.03m / 0.03s = 1m/s, future 20ms adds 0.02m
    ExpectNear(result.delta_forward_m, 0.05, 1.0e-6, "future forward prediction mismatch");
    Expect(result.predicted_ms == 20, "predicted_ms mismatch");
}

}  // namespace

int main() {
    try {
        TestEncoderForward();
        TestImuYaw();
        TestFuturePredictionDisabledFailsWhenNeeded();
        TestFutureConstantVelocity();
    } catch (const std::exception& error) {
        std::cerr << "vehicle_pose_delta_estimator_test failed: " << error.what() << "\n";
        return 1;
    }
    std::cout << "vehicle_pose_delta_estimator_test passed\n";
    return 0;
}
```

### 15.2 新增 run 脚本

路径：

```text
new/verification/tests/run_vehicle_pose_delta_estimator_test.sh
```

代码：

```bash
#!/bin/bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_BIN="${SCRIPT_DIR}/vehicle_pose_delta_estimator_test"

source "${SCRIPT_DIR}/common_steering_test_build.sh"

compile_test_binary \
  "${OUT_BIN}" \
  "${REPO_ROOT}/new/verification/tests/vehicle_pose_delta_estimator_test.cpp" \
  "${REPO_ROOT}/new/code/estimation/vehicle_pose_delta_estimator.cpp"

"${OUT_BIN}"
```

### 15.3 修改 reference_time_alignment 测试

当前测试里 reason 是：

```cpp
aligned_yaw_only
```

改成：

```cpp
aligned_measured_se2
```

新增测试：

```cpp
ls2k::port::VehiclePoseDelta MakePoseDelta(double forward,
                                           double lateral,
                                           double yaw,
                                           uint64_t start = 100,
                                           uint64_t now = 130,
                                           uint64_t end = 130) {
    ls2k::port::VehiclePoseDelta delta{};
    delta.valid = true;
    delta.reason = "test";
    delta.start_time_ms = start;
    delta.now_time_ms = now;
    delta.end_time_ms = end;
    delta.measured_until_ms = now;
    delta.delta_forward_m = forward;
    delta.delta_lateral_m = lateral;
    delta.delta_yaw_rad = yaw;
    return delta;
}

void TestForwardShift() {
    const auto path = MakeStraightPath();
    auto params = EnabledParams();

    auto result = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        130,
        MakePoseDelta(0.1, 0.0, 0.0),
        params);

    Expect(result.facts.valid, "forward SE2 alignment invalid");
    ExpectNear(result.reference_path.sampled_path[0].point.forward_m,
               0.1,
               1.0e-6,
               "forward shift mismatch");
}

void TestLateralShift() {
    const auto path = MakeStraightPath();
    auto params = EnabledParams();

    auto result = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        130,
        MakePoseDelta(0.0, 0.05, 0.0),
        params);

    Expect(result.facts.valid, "lateral SE2 alignment invalid");
    ExpectNear(result.reference_path.sampled_path[0].point.lateral_m,
               -0.05,
               1.0e-6,
               "lateral shift mismatch");
}

void TestFutureEffectiveTimeFacts() {
    const auto path = MakeStraightPath();
    auto params = EnabledParams();

    auto delta = MakePoseDelta(0.1, 0.0, 0.0, 100, 130, 150);
    delta.predicted_ms = 20;

    auto result = ls2k::reference::AlignReferencePathToVehiclePoseDelta(
        path,
        100,
        130,
        150,
        delta,
        params);

    Expect(result.facts.valid, "effective time alignment invalid");
    Expect(result.facts.control_time_ms == 130, "control_time_ms mismatch");
    Expect(result.facts.control_effective_time_ms == 150, "control_effective_time_ms mismatch");
    Expect(result.facts.predicted_ms == 20, "predicted_ms mismatch");
    Expect(result.facts.reason == "aligned_effective_se2", "reason mismatch");
}
```

### 15.4 修改 reference_time_alignment run 脚本

当前：

```bash
compile_test_binary \
  "${OUT_BIN}" \
  "${REPO_ROOT}/new/verification/tests/reference_time_alignment_test.cpp" \
  "${REPO_ROOT}/new/code/reference/reference_time_alignment.cpp" \
  "${REPO_ROOT}/new/code/port/perf_counter.cpp"
```

可以保持不变，因为 reference 模块不再依赖 estimator。

---

## 16. 第十三步：编排链最终代码形态

最终 `BuildControlTimePerception()` 应接近下面这样：

```cpp
port::PerceptionResult BuildControlTimePerception(
    const port::PerceptionResult& perception,
    const port::MotionHistory& motion_history,
    const port::ControlCommandHistory& command_history,
    uint64_t now_ms,
    const port::RuntimeParameters& params) {
    if (!perception.published || !perception.fresh || !params.reference_time_alignment.enabled) {
        port::PerceptionResult out = perception;
        out.reference_time_alignment.enabled = params.reference_time_alignment.enabled;
        out.reference_time_alignment.valid = !params.reference_time_alignment.enabled;
        out.reference_time_alignment.reason =
            params.reference_time_alignment.enabled ? "perception_unavailable" : "disabled";
        out.reference_time_alignment.reference_capture_time_ms = perception.reference_capture_time_ms;
        out.reference_time_alignment.control_time_ms = now_ms;
        out.reference_time_alignment.control_effective_time_ms =
            now_ms + static_cast<uint64_t>(
                std::max(0, params.reference_time_alignment.effective_delay_ms));
        return out;
    }

    port::PerceptionResult out = perception;

    const uint64_t effective_delay_ms =
        static_cast<uint64_t>(std::max(0, params.reference_time_alignment.effective_delay_ms));
    const uint64_t control_effective_time_ms = now_ms + effective_delay_ms;

    const port::VehiclePoseDelta pose_delta =
        estimation::EstimateVehiclePoseDelta(perception.reference_capture_time_ms,
                                             now_ms,
                                             control_effective_time_ms,
                                             motion_history,
                                             command_history,
                                             params.reference_time_alignment);

    const reference::ReferenceTimeAlignmentResult alignment =
        reference::AlignReferencePathToVehiclePoseDelta(perception.reference_path,
                                                       perception.reference_capture_time_ms,
                                                       now_ms,
                                                       control_effective_time_ms,
                                                       pose_delta,
                                                       params);

    out.reference_time_alignment = alignment.facts;
    out.reference_path = alignment.reference_path;

    if (!alignment.facts.valid) {
        out.reference_usability = {};
        out.reference_usability.reason = "reference_time_alignment_" + alignment.facts.reason;
        out.reference_lateral_error = {};
        out.reference_lateral_error.reason = out.reference_usability.reason;
        out.reference_tracking_geometry = {};
        out.reference_tracking_geometry.reason = out.reference_usability.reason;
        out.reference_control = {};
        out.reference_control.reason = out.reference_usability.reason;
        return out;
    }

    out.reference_usability = reference::EvaluateReferenceUsability(alignment.reference_path, params);
    out.reference_lateral_error =
        reference::ComputeReferenceLateralError(alignment.reference_path,
                                                out.reference_usability,
                                                params);
    out.reference_tracking_geometry =
        reference::ComputeReferenceTrackingGeometry(alignment.reference_path,
                                                    out.reference_usability,
                                                    params.bev_control_model);
    out.reference_control =
        reference::EvaluateReferenceControlReadiness(out.reference_usability,
                                                     out.reference_tracking_geometry,
                                                     perception.reference_control.degraded);
    return out;
}
```

这段代码体现最终架构：

```text
ControlLoop 知道 estimator + aligner；
estimator 不知道 path；
aligner 不知道 sensors；
controller 不知道 alignment。
```

---

## 17. 依赖规则

后续 review 时按这个表检查：

| 文件 | 允许 include | 禁止 include |
|---|---|---|
| `estimation/vehicle_pose_delta_estimator.cpp` | `motion_history_types.hpp`, `control_command_history_types.hpp`, `vehicle_pose_delta_types.hpp`, `runtime_parameter_types.hpp` | `bev_reference_types.hpp`, `reference_tracking_geometry_types.hpp`, `control/steering_yaw_controller.hpp` |
| `reference/reference_time_alignment.cpp` | `bev_reference_types.hpp`, `vehicle_pose_delta_types.hpp`, `runtime_parameter_types.hpp` | `motion_history_types.hpp`, `control_command_history_types.hpp`, `control/*` |
| `control/steering_yaw_controller.cpp` | `reference_tracking_geometry_types.hpp`, `runtime_parameter_types.hpp`, `steering_state_types.hpp` | `motion_history_types.hpp`, `vehicle_pose_delta_types.hpp`, `reference_time_alignment.hpp` |
| `runtime/loops/control_loop.cpp` | 可以 include 所有编排所需模块 | 不应把 estimator 逻辑内联进来 |

---

## 18. 分阶段上线计划

### 阶段 0：默认关闭

配置：

```json
"REFERENCE_TIME_ALIGNMENT": {
  "ENABLED": 0
}
```

目标：确保改动不影响原行为。

### 阶段 1：只启用 yaw / SE(2) 框架，不启用 encoder forward

配置：

```json
"REFERENCE_TIME_ALIGNMENT": {
  "ENABLED": 1,
  "EFFECTIVE_DELAY_MS": 0,
  "USE_ENCODER_FORWARD": 0,
  "USE_IMU_YAW": 1,
  "FUTURE_PREDICTION_ENABLED": 0
}
```

目标：验证新 API、新 facts、新 debug、新测试全部通过。

### 阶段 2：启用 encoder forward，但不预测未来

配置：

```json
"REFERENCE_TIME_ALIGNMENT": {
  "ENABLED": 1,
  "EFFECTIVE_DELAY_MS": 0,
  "USE_ENCODER_FORWARD": 1,
  "ENCODER_TICKS_TO_METER": "<实测值>",
  "USE_IMU_YAW": 1,
  "FUTURE_PREDICTION_ENABLED": 0
}
```

目标：实现：

```text
Path(t_camera) → Path(t_now)
```

### 阶段 3：启用 effective delay，使用常速未来预测

配置：

```json
"REFERENCE_TIME_ALIGNMENT": {
  "ENABLED": 1,
  "EFFECTIVE_DELAY_MS": 25,
  "FUTURE_PREDICTION_ENABLED": 1,
  "COMMAND_YAW_PREDICTION_ENABLED": 0
}
```

目标：实现：

```text
Path(t_camera) → Path(t_control_effective)
```

但未来段只使用当前测得速度和 yaw rate，不使用命令模型。

### 阶段 4：启用命令 yaw 预测

配置：

```json
"REFERENCE_TIME_ALIGNMENT": {
  "ENABLED": 1,
  "EFFECTIVE_DELAY_MS": 25,
  "FUTURE_PREDICTION_ENABLED": 1,
  "COMMAND_YAW_PREDICTION_ENABLED": 1,
  "TURN_OUTPUT_TO_YAW_RATE_GAIN": "<实测值>",
  "ACTUATOR_YAW_TAU_MS": 35.0
}
```

目标：实现最终闭环：

```text
视觉路径点
+ 当前车速
+ 当前 yaw rate
+ 历史执行命令
+ 执行响应模型
+ 系统总延迟
→ 未来车体坐标系路径
→ 原控制器
```

---

## 19. 调参和标定建议

### 19.1 `encoder_ticks_to_meter`

实测方法：

```text
车直线前进固定距离 D 米
记录左右 encoder delta 均值 T
encoder_ticks_to_meter = D / T
```

### 19.2 `effective_delay_ms`

建议拆解：

```text
camera exposure / capture delay
+ frame buffering delay
+ perception processing delay
+ control_period_ms / 2
+ actuator command transport delay
+ 差速响应达到主要效果的时间
```

初始可取：

```text
20 ~ 40 ms
```

但最终应该通过 debug 中的：

```text
reference_capture_time_ms
control_time_ms
control_effective_time_ms
predicted_ms
```

做离线回放校准。

### 19.3 `turn_output_to_yaw_rate_gain`

实验：

```text
固定速度
给定 applied_turn_output = U
测稳定 gyro_z = W rad/s
gain = W / U
```

分左右方向分别测，如果不对称，后续可以扩展为：

```cpp
double turn_output_to_yaw_rate_gain_left;
double turn_output_to_yaw_rate_gain_right;
```

---

## 20. Fail-closed 行为

最终系统必须宁可不用对齐，也不要输出错误对齐路径。

建议保留这些失败原因：

```text
perception_unavailable
invalid_reference_time
age_exceeded
pose_delta_invalid_pose_delta_time
pose_delta_motion_history_unavailable
pose_delta_encoder_scale_unavailable
pose_delta_encoder_history_unavailable
pose_delta_yaw_history_unavailable
pose_delta_future_prediction_disabled
pose_delta_future_prediction_horizon_exceeded
pose_delta_delta_forward_exceeded
pose_delta_delta_lateral_exceeded
pose_delta_delta_yaw_exceeded
aligned_samples_insufficient
```

失败后当前 `BuildControlTimePerception()` 已经会把：

```cpp
reference_usability
reference_lateral_error
reference_tracking_geometry
reference_control
```

全部置为不可用，从而让 gate / readiness 阻止控制。

---

## 21. 最终形态检查

实现完成后，代码语义应变成：

```text
PerceptionResult.reference_path:
    Path(t_camera)

VehiclePoseDeltaEstimator:
    VehiclePose(t_camera → t_control_effective)

ReferenceTimeAlignment:
    Path(t_camera) ⊕ inverse(VehiclePoseDelta)
    = Path(t_control_effective)

ReferenceTrackingGeometry:
    Fit(Path(t_control_effective))

SteeringYawController:
    Control(Fit(Path(t_control_effective)))
```

也就是：

```text
看到哪里 → 不再直接控制哪里

现在看到的路径
先被投影到控制真正生效时的车体坐标系
然后再控制。
```

---

## 22. 最小 PR 切分建议

### PR 1：纯类型和参数

```text
control_command_history_types.hpp
vehicle_pose_delta_types.hpp
runtime_parameter_types.hpp
bev_reference_types.hpp
control_debug_snapshot.hpp
param_store.cpp
default_params.json
```

不改变运行行为。

### PR 2：VehiclePoseDeltaEstimator

```text
estimation/vehicle_pose_delta_estimator.*
vehicle_pose_delta_estimator_test.cpp
run_vehicle_pose_delta_estimator_test.sh
```

不接入控制循环。

### PR 3：ReferenceTimeAlignment SE(2)

```text
reference_time_alignment.hpp
reference_time_alignment.cpp
reference_time_alignment_test.cpp
```

仍不改控制器。

### PR 4：ControlLoop 编排接入

```text
runtime_state.hpp
control_loop.cpp
```

默认配置关闭，确保行为回归。

### PR 5：上线参数和遥测

```text
default_params.json
default_params.md
debug / media serialization 若有字段映射
```

---

## 23. 最终一句话

这次实现的关键不是“给旧误差加一个延迟补偿系数”，而是把系统拆成三个互不知晓的事实变换：

```text
MotionHistory + CommandHistory → VehiclePoseDelta
BEVReferencePath + VehiclePoseDelta → AlignedBEVReferencePath
AlignedBEVReferencePath → ReferenceTrackingGeometry → 原控制器
```

这样最终系统才是：

```text
时间对齐的路径跟踪系统
```

而不是：

```text
看到哪里 → 控制哪里
```

---

## 24. 最终收尾：清理兼容字段

代码任务完成后，最后一个实现步骤不是继续保留迁移期兼容字段，而是把 active 合同收敛到当前事实名：

```text
delta_forward_m
delta_lateral_m
delta_yaw_rad
```

收尾要求：

- active `new/code/**`、`new/config/**`、`new/user/**`、`new/verification/tests/**`
  不保留旧前向位移 alias。
- debug reporter、steering media config snapshot、测试断言和参数文档都使用当前字段名。
- 若某个外部协议确实必须保留旧字段，需要先补充明确 owner、兼容期限和测试合同；否则不能作为本变更的最终状态。
- source-first review 前执行 active-surface 搜索，证明旧兼容字段没有继续作为运行合同、debug 字段、配置键或测试期望存在。
