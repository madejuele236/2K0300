/**
 * @file runtime_parameter_types.hpp
 * @brief 全部运行时参数类型定义
 *
 * 集中定义所有可在运行时动态调整的参数。
 * 包含车轮PID、助理TCP、运动控制、感知、BEV投影、转向媒体、低电压采样等全部子系统参数。
 * 参数通过JSON配置文件加载，支持启动时批量下发。
 */

#ifndef LS2K_PORT_RUNTIME_PARAMETER_TYPES_HPP
#define LS2K_PORT_RUNTIME_PARAMETER_TYPES_HPP

#include <string>

#include "port/actuator_command_types.hpp"
#include "port/bev_geometry_types.hpp"
#include "port/visual_element_evidence_types.hpp"

namespace ls2k::port {

/// TCP protocol port domain shared by assistant and steering-media endpoints.
inline constexpr int kMinimumTcpPort = 1;
inline constexpr int kMaximumTcpPort = 65535;

/// ParamStore policy for signed-millisecond scheduling and state-machine
/// windows: services use explicit enable flags or documented zero-duration
/// semantics, not INT_MAX sentinels. One day is the largest supported single
/// finite-run window; larger values are configuration errors.
inline constexpr int kMaximumRuntimeIntervalMs = 24 * 60 * 60 * 1000;

/// The control timer drives safety-gate and actuator decisions. The supported
/// control contract therefore requires at least one decision per second.
inline constexpr int kMaximumControlPeriodMs = 1000;

/**
 * @struct WheelPidParameters
 * @brief 车轮PID控制器参数
 *
 * 左右轮可独立配置PID参数和测量滤波系数。
 */
struct WheelPidParameters {
    double p = 84.0;             ///< 有限比例增益
    double i = 2.4;              ///< 有限积分增益
    double d = 0.75;             ///< 有限微分增益
    double integral_limit = 5000.0;  ///< 非负有限积分项限幅
    double measurement_filter_alpha = 0.4;  ///< 有限低通系数：[0,1]；0=保持历史，1=完全采用新值
};

/**
 * @struct AssistantTcpParameters
 * @brief 助理连接的TCP参数
 *
 * 定义与地面站/上位机通信的TCP连接配置。
 */
struct AssistantTcpParameters {
    std::string host = "192.168.137.1";   ///< 上位机IP地址
    int port = 48011;                     ///< TCP端口号，[1,65535]
};

/// Reference time alignment 参数
struct ReferenceTimeAlignmentParameters {
    bool enabled = false;                  ///< 是否启用控制侧 reference 时间对齐
    int max_age_ms = 120;                  ///< 最大可对齐 reference 年龄
    int effective_delay_ms = 0;            ///< now -> control effective 的估计延迟
    int future_prediction_max_ms = 80;     ///< 允许预测的最大未来时间
    int max_integration_gap_ms = 30;       ///< motion history 最大允许采样空洞
    int min_aligned_samples = 3;           ///< 对齐后最少前方样本数
    bool use_encoder_forward = false;      ///< 是否用编码器积分前进距离
    double wheel_track_m = 0.154;          ///< 左右轮中心距，用于轮速 yaw fallback
    bool use_imu_yaw = true;               ///< 是否使用 IMU gyro_z 积分 yaw
    bool use_wheel_yaw_fallback = false;   ///< IMU 不可用时是否用左右轮差估 yaw
    bool future_prediction_enabled = false;      ///< 是否预测 now -> effective
    bool command_yaw_prediction_enabled = false; ///< 是否用 applied turn 输出预测 yaw
    double turn_output_to_yaw_rate_gain = 0.0;   ///< turn output -> yaw rate(rad/s)
    double actuator_yaw_tau_ms = 35.0;           ///< yaw 响应一阶时间常数
    double max_delta_forward_m = 0.782641706;  ///< 单次对齐最大前向位移
    double max_delta_lateral_m = 0.441152591;  ///< 单次对齐最大横向位移
    double max_delta_yaw_rad = 0.80;        ///< 单次对齐最大 yaw 积分
};

/// Camera source 参数，独立于 BEV/circle/cross 语义
struct CameraSourceParameters {
    std::string backend = "v4l2_yuyv";     ///< 主相机后端
    std::string device = "/dev/video0";    ///< V4L2 设备路径
    int width = 320;                       ///< 采集宽度
    int height = 240;                      ///< 采集高度
    int fps = 60;                          ///< 目标帧率
    int buffer_count = 3;                  ///< V4L2 mmap buffer 数
    int poll_timeout_ms = 50;              ///< capture thread 内 poll 超时
    bool drain_ready_buffers = true;       ///< 在 mmap buffer 数量上界内 drain 到最新 ready buffer
};

struct MotionOdometryParameters {
    double encoder_ticks_to_meter = 0.0;
};

struct MlRoiParameters {
    double search_forward_min_m = 0.0;
    double search_forward_max_m = 0.0;
    double search_lateral_limit_m = 0.0;
    double grid_forward_step_m = 0.0;
    double grid_lateral_step_m = 0.0;
    int red_y_min = 0;
    int red_y_max = 255;
    int red_u_min = 0;
    int red_u_max = 255;
    int red_v_min = 0;
    int red_v_max = 255;
    double expected_long_edge_m = 0.0;
    double expected_short_edge_m = 0.0;
    double crop_long_offset_m = 0.0;
    double crop_forward_offset_m = 0.0;
    double long_edge_tolerance_m = 0.0;
    double short_edge_tolerance_m = 0.0;
    double max_long_edge_to_lateral_rad = 0.0;
    int min_component_cells = 0;
    double min_rectangularity = 0.0;
    double min_red_fill_ratio = 0.0;
    double score_size_weight = 1.0;
    double score_rectangularity_weight = 1.0;
    double score_red_fill_weight = 1.0;
    double score_orientation_weight = 1.0;
};

struct MlV9Parameters {
    int min_margin = 0;
    int max_best_distance = 126;
    int confirm_frames = 1;
};

struct MlTfliteIdentityParameters {
    int min_margin = 1;
    int max_best_distance = 2076;
    int confirm_frames = 3;
};

struct MlClassMappingParameters {
    std::string class_0_action = "straight";
    std::string class_1_action = "left";
    std::string class_2_action = "right";
};

struct MlManeuverParameters {
    bool enabled = false;        ///< 是否允许 ML 事实进入路径仲裁和速度策略
    double speed_target = 0.0;  ///< maneuver 启用时为 (0,5000] 的 wheel mixer base
    int min_boundary_samples = 3;
    double path_outward_offset_m = 0.0;
    double exit_forward_m = 0.0;
    int max_duration_ms = 0;
    int max_integration_gap_ms = 0;
    int cooldown_ms = 0;
};

struct MlParameters {
    bool enabled = false;
    MlRoiParameters roi{};
    MlV9Parameters v9{};
    MlTfliteIdentityParameters tflite_identity{};
    MlClassMappingParameters class_mapping{};
    MlManeuverParameters maneuver{};
};

/**
 * @struct RuntimeParameters
 * @brief 完整的运行时参数集合
 *
 * 涵盖所有子系统的运行参数，从JSON配置文件加载后供各模块查询使用。
 * 包含运动控制（PID增益、PWM限幅、运动超时等）、感知（相机尺寸、分类阈值）、
 * 通信（助理TCP、转向媒体服务）、电源监控等全部可调参数。
 */
struct RuntimeParameters {
    // 运动控制参数
    double running_speed_target = 300.0;  ///< wheel mixer base 目标，与启用的 ML 速度共用 [0,5000] 合同
    double yaw_rate_pid_p = 0.0;          ///< 经生产 float 最坏 error 算术验证的偏航比例增益
    double yaw_rate_pid_i = 0.0;          ///< 经 |积分累加器|<=1200 算术验证的偏航积分增益
    double yaw_rate_pid_d = 0.0;          ///< 经生产最大 error delta 算术验证的偏航微分增益
    // 安全与低电压
    int low_voltage_raw_threshold = 200; ///< 低电压原始阈值

    // 控制周期与超时
    int control_period_ms = 5;            ///< 控制周期（毫秒），[1,1000]
    int perception_stale_ms = 120;        ///< 感知数据过期阈值（毫秒），[1,24h]

    // 电机PWM限制
    int pwm_limit = kDrivePwmDutyCapability;  ///< PWM最大绝对值，不得超过驱动硬件能力
    int raw_turn_output_limit = 20000;    ///< 非负转向输出二次限幅（还受生产硬上界 9000 约束）
    double wheel_turn_accel_delta_scale = 2.0;  ///< 非负有限，且最大 base + |turn|*scale 必须有限
    double wheel_turn_decel_delta_scale = 1.0;  ///< 非负有限，且 |turn|*scale 必须有限
    int pwm_floor = 0;                    ///< PWM最低有效值，必须在 [0, pwm_limit] 内
    bool prohibit_reverse_pwm = true;     ///< 是否禁止反转PWM
    int drive_pwm_step_limit = 1000;      ///< 左右有刷驱动 PWM 每控制周期最大变化量
    bool brushless_debug_fixed_pwm_enabled = false;  ///< 是否启用无刷电调固定 PWM 调试输出
    int brushless_debug_fixed_pwm = 1000;   ///< 无刷电调固定 PWM 调试输出值（0~1000）

    // 运动状态机参数
    int motion_unveto_confirm_cycles = 3;   ///< 解除封锁需要的确认周期数，至少 1
    int motion_spinup_ms = 800;             ///< 电机启动加速时间（毫秒），[0,24h]
    double motion_turn_limit_spinup = 1.0;  ///< 启动阶段的有限转向比例，[0,1]
    int motion_stop_ms = 300;               ///< 停止超时时间（毫秒），[0,24h]
    int motion_stop_encoder_threshold = 8;  ///< 停止判定编码器阈值，[0,5000]
    int motion_fault_rearm_hold_ms = 600;   ///< 故障后重新就绪等待时间（毫秒），[0,24h]

    // 左右轮独立PID
    WheelPidParameters left_wheel_pid{};    ///< 左轮PID参数
    WheelPidParameters right_wheel_pid{96.0, 2.2, 0.2, 5000.0, 0.4};  ///< 右轮PID参数

    // 调试与通信
    int control_snapshot_emit_interval_ms = 100;  ///< 控制快照输出间隔（毫秒），[1,24h]
    bool assistant_enabled = true;                ///< 是否启用助理连接
    AssistantTcpParameters assistant_tcp{};        ///< 助理TCP参数
    bool steering_media_enabled = true;            ///< 是否启用转向媒体服务
    int steering_media_port = 48012;               ///< 媒体服务端口，[1,65535]
    int steering_media_publish_interval_ms = 20;   ///< 媒体发布间隔（毫秒），[0,24h]；0=每次 eligible tick
    int steering_media_downsample = 1;             ///< 媒体图像下采样因子
    bool steering_media_publish_latest_frame = false;  ///< 诊断开关：优先发布最新相机帧而非严格快照匹配帧
    int steering_media_gray_bits = 2;               ///< 媒体图像灰度位深，支持 1/2/4/8
    bool steering_media_publish_disarmed = true;   ///< 媒体发布是否处于未就绪状态
    int low_voltage_sample_interval_ms = 1000;     ///< 低电压采样间隔（毫秒），[1,24h]

    // BEV参数
    BEVProjectorCalibration bev_projector{};             ///< BEV投影器标定参数
    BEVGeometryParameters bev_geometry{};                 ///< BEV几何参数
    BEVClassificationParameters bev_classification{};     ///< BEV分类参数
    BEVControlModelParameters bev_control_model{};        ///< BEV控制模型参数
    BEVElementParameters bev_element{};                   ///< BEV元素检测参数
    ReferenceTimeAlignmentParameters reference_time_alignment{};  ///< 参考时间对齐参数
    CameraSourceParameters camera_source{};                       ///< 相机源参数
    MotionOdometryParameters motion_odometry{};
    MlParameters ml{};

    // 状态标记
    bool loaded_from_defaults = false;      ///< 是否从默认值加载（非JSON文件）
    bool parse_failure = false;             ///< JSON解析是否失败
};

}  // namespace ls2k::port

#endif  // LS2K_PORT_RUNTIME_PARAMETER_TYPES_HPP
