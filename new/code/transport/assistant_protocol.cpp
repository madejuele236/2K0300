#include "transport/assistant_protocol.hpp"

// 辅助协议实现 —— 外部助手（如远程控制台）的 JSON 命令解析和状态编码。
// 支持命令：start/stop、tuning mode、turn suppression、target speed override。

#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

#include <opencv2/core/persistence.hpp>

#include "transport/visual_element_evidence_json.hpp"

namespace ls2k::transport {
namespace {

constexpr std::size_t kMaxInboundLineBytes = 4096;

/// @brief 尝试将文本解析为 JSON 对象
/// @param text 输入 JSON 文本
/// @param storage 输出参数：解析后的 FileStorage 对象
/// @return 解析成功且根节点为非空 Map 时返回 true
bool ParseJsonObject(const std::string& text, cv::FileStorage& storage) {
    try {
        if (!storage.open(text,
                          cv::FileStorage::READ | cv::FileStorage::MEMORY |
                              cv::FileStorage::FORMAT_JSON)) {
            return false;
        }
    } catch (...) {
        return false;
    }
    const cv::FileNode root = storage.root();
    return !root.empty() && root.isMap();
}

/// @brief 从 JSON 节点读取字符串值
/// @param node JSON 节点
/// @param value 输出参数：读取的字符串
/// @return 读取成功返回 true
bool ReadStringValue(const cv::FileNode& node, std::string& value) {
    if (node.empty() || !node.isString()) {
        return false;
    }
    value = static_cast<std::string>(node);
    return true;
}

/// @brief 从 JSON 节点读取有限数值（非 NaN / Inf）
/// @param node JSON 节点
/// @param value 输出参数：读取的 double 值
/// @return 读取成功且为有限数值时返回 true
bool ReadFiniteNumber(const cv::FileNode& node, double& value) {
    if (node.empty() || (!node.isInt() && !node.isReal())) {
        return false;
    }
    value = static_cast<double>(node.real());
    return std::isfinite(value);
}

/// @brief 从 JSON 节点读取非负整数值
/// @param node JSON 节点
/// @param value 输出参数：读取的 uint64 值
/// @return 读取成功且为非负整数时返回 true
bool ReadNonNegativeInteger(const cv::FileNode& node, std::uint64_t& value) {
    double numeric = 0.0;
    if (!ReadFiniteNumber(node, numeric) || numeric < 0.0) {
        return false;
    }
    const double rounded = std::round(numeric);
    if (std::fabs(numeric - rounded) > 1e-6) {
        return false;
    }
    if (rounded > static_cast<double>(std::numeric_limits<std::uint64_t>::max())) {
        return false;
    }
    value = static_cast<std::uint64_t>(rounded);
    return true;
}

/// @brief 从 JSON 节点读取正整数（值为 int 且 > 0）
/// @param node JSON 节点
/// @param value 输出参数：读取的 int 值
/// @return 读取成功且为正整数时返回 true
bool ReadPositiveInt(const cv::FileNode& node, int& value) {
    std::uint64_t parsed = 0;
    if (!ReadNonNegativeInteger(node, parsed) || parsed == 0 ||
        parsed > static_cast<std::uint64_t>(std::numeric_limits<int>::max())) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

/// @brief 从 JSON 节点读取布尔值
/// @param node JSON 节点
/// @param value 输出参数：读取的 bool 值
/// @return 读取成功返回 true
bool ReadBoolValue(const cv::FileNode& node, bool& value) {
    if (node.empty() || !node.isInt()) {
        return false;
    }
    const int numeric = static_cast<int>(node.real());
    if (numeric == 0 || numeric == 1) {
        value = numeric != 0;
        return true;
    }
    return false;
}

/// @brief 构造"输入被拒"消息
/// @param reason 拒绝原因
/// @return 构造好的 AssistantInboundMessage
AssistantInboundMessage MakeInputRejected(std::string reason) {
    AssistantInboundMessage message{};
    message.type = AssistantInboundMessageType::kInputRejected;
    message.reason = std::move(reason);
    return message;
}

/// @brief 构造"命令确认被拒"消息
/// @param seq 被拒命令的序列号
/// @param reason 拒绝原因
/// @return 构造好的 AssistantInboundMessage
AssistantInboundMessage MakeAckRejected(std::uint64_t seq, std::string reason) {
    AssistantInboundMessage message{};
    message.type = AssistantInboundMessageType::kAckRejected;
    message.seq = seq;
    message.reason = std::move(reason);
    return message;
}

/// @brief 构造命令消息
/// @param command 命令结构体
/// @return 构造好的 AssistantInboundMessage
AssistantInboundMessage MakeCommand(AssistantCommand command) {
    AssistantInboundMessage message{};
    message.type = AssistantInboundMessageType::kCommand;
    message.command = command;
    return message;
}

/// @brief 将字符串值以 JSON 格式追加到输出流（含转义处理）
/// @param stream 输出流
/// @param value 要编码的字符串
void AppendJsonString(std::ostringstream& stream, const std::string& value) {
    stream << '"';
    for (const char ch : value) {
        switch (ch) {
            case '\\':
                stream << "\\\\";
                break;
            case '"':
                stream << "\\\"";
                break;
            case '\n':
                stream << "\\n";
                break;
            case '\r':
                stream << "\\r";
                break;
            case '\t':
                stream << "\\t";
                break;
            default:
                if (static_cast<unsigned char>(ch) < 0x20U) {
                    stream << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                           << static_cast<int>(static_cast<unsigned char>(ch)) << std::dec
                           << std::setfill(' ');
                } else {
                    stream << ch;
                }
                break;
        }
    }
    stream << '"';
}

/// @brief 将双精度数值以 JSON 格式追加到输出流
/// @param stream 输出流
/// @param value 要编码的数值
void AppendJsonNumber(std::ostringstream& stream, double value) {
    stream << std::setprecision(12) << value;
}

/// @brief 将布尔值以 JSON 格式追加到输出流
/// @param stream 输出流
/// @param value 要编码的布尔值
void AppendJsonBool(std::ostringstream& stream, bool value) {
    stream << (value ? "true" : "false");
}

void AppendCircleOpeningJson(std::ostringstream& stream,
                             const port::CircleOpeningObservation& opening) {
    stream << "{\"available\":";
    AppendJsonBool(stream, opening.available);
    stream << ",\"frontier_forward_m\":";
    if (opening.available) {
        AppendJsonNumber(stream, opening.frontier_forward_m);
    } else {
        stream << "null";
    }
    stream << ",\"effective_lateral_m\":";
    if (opening.available) {
        AppendJsonNumber(stream, opening.effective_lateral_m);
    } else {
        stream << "null";
    }
    stream << ",\"source\":";
    AppendJsonString(stream, port::CircleOpeningSourceToken(opening.source));
    stream << ",\"outward_distance_m\":";
    if (opening.available) {
        AppendJsonNumber(stream, opening.outward_distance_m);
    } else {
        stream << "null";
    }
    stream << ",\"confirmed_forward_span_m\":";
    if (opening.available) {
        AppendJsonNumber(stream, opening.confirmed_forward_span_m);
    } else {
        stream << "null";
    }
    stream << ",\"origin_connected\":";
    AppendJsonBool(stream, opening.origin_connected);
    stream << ",\"opposite_straight\":";
    AppendJsonBool(stream, opening.opposite_straight);
    stream << "}";
}

void AppendCircleV2TelemetryJson(std::ostringstream& stream,
                                 const port::CircleV2TelemetrySnapshot& circle) {
    stream << "{\"enabled\":";
    AppendJsonBool(stream, circle.enabled);
    stream << ",\"frame_phase\":";
    AppendJsonString(stream, circle.frame_phase);
    stream << ",\"next_phase\":";
    AppendJsonString(stream, circle.next_phase);
    stream << ",\"dir\":";
    AppendJsonString(stream, circle.dir);
    stream << ",\"reference_role\":";
    AppendJsonString(stream, circle.reference_role);
    stream << ",\"reason\":";
    AppendJsonString(stream, circle.reason);
    stream << ",\"motion_arc_available\":";
    AppendJsonBool(stream, circle.motion_arc_available);
    stream << ",\"geometry_available\":";
    AppendJsonBool(stream, circle.geometry_available);
    stream << ",\"inner_trace_elapsed_ms\":" << circle.inner_trace_elapsed_ms;
    stream << ",\"directed_turn_angle_rad\":";
    AppendJsonNumber(stream, circle.directed_turn_angle_rad);
    stream << ",\"openings\":{\"left\":";
    AppendCircleOpeningJson(stream, circle.openings.left);
    stream << ",\"right\":";
    AppendCircleOpeningJson(stream, circle.openings.right);
    stream << "}}";
}

void AppendMlTelemetryJson(std::ostringstream& stream,
                           const port::MlTelemetrySnapshot& ml,
                           const std::string& speed_selection_source,
                           double effective_speed_target) {
    stream << "{\"enabled\":";
    AppendJsonBool(stream, ml.enabled);
    stream << ",\"maneuver_enabled\":";
    AppendJsonBool(stream, ml.maneuver_enabled);
    stream << ",\"takeover_selected\":";
    AppendJsonBool(stream, ml.takeover_selected);
    stream << ",\"artifact\":{\"candidate_id\":";
    AppendJsonString(stream, ml.artifact_candidate_id == nullptr
                                 ? "unavailable" : ml.artifact_candidate_id);
    stream << ",\"descriptor_config_hash\":";
    AppendJsonString(stream, ml.descriptor_config_hash == nullptr
                                 ? "unavailable" : ml.descriptor_config_hash);
    stream << ",\"template_table_hash\":";
    AppendJsonString(stream, ml.template_table_hash == nullptr
                                 ? "unavailable" : ml.template_table_hash);
    stream << ",\"template_codes_sha256\":";
    AppendJsonString(stream, ml.template_codes_sha256 == nullptr
                                 ? "unavailable" : ml.template_codes_sha256);
    stream << ",\"prototype_count\":" << ml.artifact_prototype_count << "}";
    stream << ",\"timing_us\":{\"detector\":" << ml.detector_us;
    stream << ",\"roi\":" << ml.roi_us;
    stream << ",\"descriptor\":" << ml.descriptor_us;
    stream << ",\"replay\":" << ml.replay_us;
    stream << ",\"classifier\":" << ml.classifier_us;
    stream << ",\"total\":" << ml.total_us << "}";
    stream << ",\"detector_valid\":";
    AppendJsonBool(stream, ml.detector_valid);
    stream << ",\"detector\":{\"valid\":";
    AppendJsonBool(stream, ml.detector.valid);
    stream << ",\"frame_id\":" << ml.detector.frame_id;
    stream << ",\"capture_time_ms\":" << ml.detector.capture_time_ms;
    stream << ",\"center\":{\"forward_m\":";
    AppendJsonNumber(stream, ml.detector.center.forward_m);
    stream << ",\"lateral_m\":";
    AppendJsonNumber(stream, ml.detector.center.lateral_m);
    stream << "},\"long_edge_m\":";
    AppendJsonNumber(stream, ml.detector.long_edge_m);
    stream << ",\"short_edge_m\":";
    AppendJsonNumber(stream, ml.detector.short_edge_m);
    stream << ",\"long_axis_forward\":";
    AppendJsonNumber(stream, ml.detector.long_axis_forward);
    stream << ",\"long_axis_lateral\":";
    AppendJsonNumber(stream, ml.detector.long_axis_lateral);
    stream << ",\"long_edge_to_lateral_rad\":";
    AppendJsonNumber(stream, ml.detector.long_edge_to_lateral_rad);
    stream << ",\"corners\":[";
    for (std::size_t index = 0; index < ml.detector.corners.size(); ++index) {
        if (index > 0U) stream << ",";
        stream << "{\"forward_m\":";
        AppendJsonNumber(stream, ml.detector.corners[index].forward_m);
        stream << ",\"lateral_m\":";
        AppendJsonNumber(stream, ml.detector.corners[index].lateral_m);
        stream << "}";
    }
    stream << "]";
    stream << ",\"rectangularity\":";
    AppendJsonNumber(stream, ml.detector.rectangularity);
    stream << ",\"red_fill_ratio\":";
    AppendJsonNumber(stream, ml.detector.red_fill_ratio);
    stream << ",\"score\":";
    AppendJsonNumber(stream, ml.detector.quality);
    stream << ",\"component_cells\":" << ml.detector.component_cells << "}";
    stream << ",\"roi\":{\"valid\":";
    AppendJsonBool(stream, ml.roi.valid);
    stream << ",\"frame_id\":" << ml.roi.frame_id;
    stream << ",\"reason\":";
    AppendJsonString(stream, ml.roi.reason == nullptr ? "unknown" : ml.roi.reason);
    stream << ",\"long_axis_forward\":";
    AppendJsonNumber(stream, ml.roi.long_axis_forward);
    stream << ",\"long_axis_lateral\":";
    AppendJsonNumber(stream, ml.roi.long_axis_lateral);
    stream << ",\"forward_normal_forward\":";
    AppendJsonNumber(stream, ml.roi.forward_normal_forward);
    stream << ",\"forward_normal_lateral\":";
    AppendJsonNumber(stream, ml.roi.forward_normal_lateral);
    stream << ",\"width\":" << port::kMlRoiSide;
    stream << ",\"height\":" << port::kMlRoiSide;
    stream << ",\"pixel_format\":\"gray8\"}";
    stream << ",\"descriptor_valid\":";
    AppendJsonBool(stream, ml.descriptor.valid);
    stream << ",\"replay\":{\"valid\":";
    AppendJsonBool(stream, ml.replay.valid);
    stream << ",\"class_id\":" << ml.replay.class_id;
    stream << ",\"best_distance\":" << ml.replay.best_distance;
    stream << ",\"margin\":" << ml.replay.margin;
    stream << ",\"prototype_index\":" << ml.replay.prototype_index << "}";
    stream << ",\"classification\":{\"valid\":";
    AppendJsonBool(stream, ml.classification.valid);
    stream << ",\"backend\":";
    AppendJsonString(stream, port::MlClassifierBackendToken(ml.classification.backend));
    stream << ",\"class_id\":" << ml.classification.class_id;
    stream << ",\"margin\":" << ml.classification.margin;
    stream << ",\"distance_valid\":";
    AppendJsonBool(stream, ml.classification.distance_valid);
    stream << ",\"best_distance\":" << ml.classification.best_distance;
    stream << ",\"scores\":[" << ml.classification.class_scores[0] << ','
           << ml.classification.class_scores[1] << ','
           << ml.classification.class_scores[2] << "]}";
    stream << ",\"mapped_action\":";
    AppendJsonString(stream, port::MlActionToken(ml.mapped_action));
    stream << ",\"locked_action\":";
    AppendJsonString(stream, port::MlActionToken(ml.locked_action));
    stream << ",\"phase\":";
    AppendJsonString(stream, port::MlScenePhaseToken(ml.phase));
    stream << ",\"reason\":";
    AppendJsonString(stream, ml.reason == nullptr ? "unknown" : ml.reason);
    stream << ",\"confirm_count\":" << ml.confirm_count;
    stream << ",\"active\":";
    AppendJsonBool(stream, ml.active);
    stream << ",\"odometry\":{\"valid\":";
    AppendJsonBool(stream, ml.odometry_valid);
    stream << ",\"reason\":";
    AppendJsonString(stream, ml.odometry_reason == nullptr ? "unknown" : ml.odometry_reason);
    stream << "}";
    stream << ",\"traveled_forward_m\":";
    AppendJsonNumber(stream, ml.traveled_forward_m);
    stream << ",\"elapsed_ms\":" << ml.elapsed_ms;
    stream << ",\"path_sample_count\":" << ml.path_sample_count;
    stream << ",\"speed_selection\":{\"source\":";
    AppendJsonString(stream, speed_selection_source);
    stream << ",\"effective_speed_target\":";
    AppendJsonNumber(stream, effective_speed_target);
    stream << "}}";
}

}  // namespace

AssistantProtocolDecoder::AssistantProtocolDecoder(double max_target_speed)
    : max_target_speed_(max_target_speed) {}

std::vector<AssistantInboundMessage> AssistantProtocolDecoder::PushBytes(
    const std::uint8_t* bytes,
    std::size_t length) {
    std::vector<AssistantInboundMessage> messages;
    if (bytes == nullptr || length == 0U) {
        return messages;
    }

    pending_bytes_.append(reinterpret_cast<const char*>(bytes), length);
    while (true) {
        const std::size_t newline_index = pending_bytes_.find('\n');
        if (newline_index == std::string::npos) {
            break;
        }

        std::string line = pending_bytes_.substr(0, newline_index);
        pending_bytes_.erase(0, newline_index + 1U);
        if (!line.empty() && line.back() == '\r') {
            line.pop_back();
        }
        messages.push_back(DecodeAssistantJsonLine(line, max_target_speed_));
    }

    if (pending_bytes_.size() > kMaxInboundLineBytes) {
        messages.push_back(AssistantInboundMessage{AssistantInboundMessageType::kInputRejected,
                                                   {},
                                                   0,
                                                   "input line too long"});
        pending_bytes_.clear();
    }
    return messages;
}

void AssistantProtocolDecoder::Reset() {
    pending_bytes_.clear();
}

std::string EncodeAssistantJsonFrame(const std::string& json) {
    std::string frame = json;
    frame.push_back('\n');
    return frame;
}

/// @brief 解码一行 JSON 格式的助手入站消息
/// @param line 原始 JSON 行字符串
/// @param max_target_speed 最大允许目标速度，用于校验速度指令
/// @return 解码后的入站消息结构体
// NOLINTNEXTLINE(readability-function-size): protocol parser keeps field validation in one JSON decoding pass.
AssistantInboundMessage DecodeAssistantJsonLine(const std::string& line, double max_target_speed) {
    if (line.empty()) {
        return MakeInputRejected("empty command line");
    }

    cv::FileStorage json;
    if (!ParseJsonObject(line, json)) {
        return MakeInputRejected("malformed json");
    }

    const cv::FileNode root = json.root();
    std::string frame_type;
    if (!ReadStringValue(root["type"], frame_type) || frame_type != "command") {
        return MakeInputRejected("unsupported frame type");
    }

    std::string cmd;
    if (!ReadStringValue(root["cmd"], cmd)) {
        return MakeInputRejected("missing command name");
    }

    std::uint64_t seq = 0;
    if (!ReadNonNegativeInteger(root["seq"], seq)) {
        return MakeInputRejected("invalid or missing seq");
    }

    if (cmd == "start") {
        AssistantCommand command{};
        command.type = AssistantCommandType::kStart;
        command.seq = seq;
        return MakeCommand(command);
    }
    if (cmd == "stop") {
        AssistantCommand command{};
        command.type = AssistantCommandType::kStop;
        command.seq = seq;
        return MakeCommand(command);
    }
    if (cmd == "enable_tuning_mode") {
        AssistantCommand command{};
        command.type = AssistantCommandType::kEnableTuningMode;
        command.seq = seq;
        return MakeCommand(command);
    }
    if (cmd == "disable_tuning_mode") {
        AssistantCommand command{};
        command.type = AssistantCommandType::kDisableTuningMode;
        command.seq = seq;
        return MakeCommand(command);
    }
    if (cmd == "set_turn_suppressed") {
        bool value = false;
        if (!ReadBoolValue(root["value"], value)) {
            return MakeAckRejected(seq, "invalid turn suppression value: expected boolean");
        }
        AssistantCommand command{};
        command.type = AssistantCommandType::kSetTurnSuppressed;
        command.seq = seq;
        command.bool_value = value;
        return MakeCommand(command);
    }
    if (cmd == "set_target_speed") {
        double value = 0.0;
        const cv::FileNode value_node = root["value"];
        if (value_node.empty()) {
            return MakeAckRejected(seq, "invalid target speed: missing value");
        }
        if (!ReadFiniteNumber(value_node, value)) {
            return MakeAckRejected(seq, "invalid target speed: value must be a finite number");
        }
        if (value < 0.0) {
            return MakeAckRejected(seq, "invalid target speed: value must be >= 0");
        }
        if (value > max_target_speed) {
            return MakeAckRejected(seq, "invalid target speed: value exceeds running_speed_target");
        }

        int ttl_ms = 0;
        if (root["ttl_ms"].empty()) {
            return MakeAckRejected(seq, "invalid target speed TTL: missing ttl_ms");
        }
        if (!ReadPositiveInt(root["ttl_ms"], ttl_ms)) {
            return MakeAckRejected(seq,
                                   "invalid target speed TTL: ttl_ms must be a positive integer");
        }

        AssistantCommand command{};
        command.type = AssistantCommandType::kSetTargetSpeed;
        command.seq = seq;
        command.target_speed_value = value;
        command.ttl_ms = ttl_ms;
        return MakeCommand(command);
    }

    return MakeInputRejected("unsupported command");
}

/// @brief 编码助手应答（ACK/NAK）JSON 消息
/// @param seq 对应命令的序列号
/// @param accepted 是否接受
/// @param reason 拒绝原因（仅 accepted=false 时使用）
/// @return 编码后的 JSON 字符串
std::string EncodeAssistantAck(std::uint64_t seq, bool accepted, const std::string& reason) {
    std::ostringstream stream;
    stream << "{\"type\":\"ack\",\"seq\":" << seq << ",\"outcome\":\""
           << (accepted ? "accepted" : "rejected") << '"';
    if (!accepted) {
        stream << ",\"reason\":";
        AppendJsonString(stream, reason);
    }
    stream << '}';
    return stream.str();
}

/// @brief 编码助手状态变更 JSON 消息
/// @param event 事件名称
/// @param reason 事件原因
/// @param status 当前状态快照
/// @return 编码后的 JSON 字符串
std::string EncodeAssistantState(const std::string& event,
                                 const std::string& reason,
                                 const AssistantStatusView& status) {
    std::ostringstream stream;
    stream << "{\"type\":\"state\",\"event\":";
    AppendJsonString(stream, event);
    stream << ",\"reason\":";
    AppendJsonString(stream, reason);
    stream << ",\"tuning_mode_enabled\":";
    AppendJsonBool(stream, status.tuning_mode_enabled);
    stream << ",\"turn_suppressed\":";
    AppendJsonBool(stream, status.turn_suppressed);
    stream << ",\"target_speed_override_enabled\":";
    AppendJsonBool(stream, status.target_speed_override_enabled);
    stream << ",\"target_speed_override_value\":";
    if (status.target_speed_override_enabled) {
        AppendJsonNumber(stream, status.target_speed_override_value);
    } else {
        stream << "null";
    }
    stream << ",\"effective_speed_target\":";
    AppendJsonNumber(stream, status.effective_speed_target);
    stream << '}';
    return stream.str();
}

/// @brief 编码完整助手遥测 JSON 消息
/// @param telemetry 遥测数据视图
/// @return 编码后的 JSON 字符串
// NOLINTNEXTLINE(readability-function-size)
std::string EncodeAssistantTelemetry(const AssistantTelemetryView& telemetry) {
    std::ostringstream stream;
    stream << "{\"type\":\"telemetry\",\"motion_phase\":";
    AppendJsonString(stream, telemetry.motion_phase);
    stream << ",\"perception_tag\":";
    AppendJsonString(stream, telemetry.perception_tag);
    stream << ",\"otsu\":{\"valid\":";
    AppendJsonBool(stream, telemetry.otsu.valid);
    stream << ",\"threshold\":" << telemetry.otsu.threshold;
    stream << ",\"source\":";
    AppendJsonString(stream, port::ToString(telemetry.otsu.source));
    stream << ",\"stale_frames\":"
           << static_cast<unsigned int>(telemetry.otsu.stale_frames) << "}";
    stream << ",\"boundary_row_count\":" << telemetry.boundary_row_count;
    stream << ",\"boundary_jump_count\":" << telemetry.boundary_jump_count;
    stream << ",\"boundary_span_count\":" << telemetry.boundary_span_count;
    stream << ",\"ml\":";
    AppendMlTelemetryJson(stream,
                          telemetry.ml,
                          telemetry.speed_selection_source,
                          telemetry.effective_speed_target);
    stream << ",\"perception_health\":{\"projector_ok\":";
    AppendJsonBool(stream, telemetry.perception_health.projector_ok);
    stream << ",\"reason\":";
    AppendJsonString(stream, telemetry.perception_health.reason);
    stream << "}";
    stream << ",\"element_evidence\":";
    AppendVisualElementEvidenceJson(stream, telemetry.element_evidence);
    stream << ",\"circle_v2\":";
    AppendCircleV2TelemetryJson(stream, telemetry.circle_v2);
    stream << ",\"visual_reference\":{\"present\":";
    AppendJsonBool(stream, telemetry.visual_reference.present);
    stream << ",\"source\":";
    AppendJsonString(stream, telemetry.visual_reference.source);
    stream << ",\"reason\":";
    AppendJsonString(stream, telemetry.visual_reference.reason);
    stream << ",\"candidate_count\":" << telemetry.visual_reference.candidate_count;
    stream << ",\"rejected_candidate_reason\":";
    AppendJsonString(stream, telemetry.visual_reference.rejected_candidate_reason);
    stream << "}";
    stream << ",\"reference\":{\"mode\":";
    AppendJsonString(stream, telemetry.reference.mode);
    stream << ",\"source\":";
    AppendJsonString(stream, telemetry.reference.source);
    stream << "}";
    stream << ",\"eligibility\":{\"usable\":";
    AppendJsonBool(stream, telemetry.eligibility.usable);
    stream << ",\"leading_usable_samples\":" << telemetry.eligibility.leading_usable_samples;
    stream << ",\"leading_min_forward_m\":";
    AppendJsonNumber(stream, telemetry.eligibility.leading_min_forward_m);
    stream << ",\"leading_max_forward_m\":";
    AppendJsonNumber(stream, telemetry.eligibility.leading_max_forward_m);
    stream << ",\"reason\":";
    AppendJsonString(stream, telemetry.eligibility.reason);
    stream << "}";
    stream << ",\"lateral_error\":{\"computed\":";
    AppendJsonBool(stream, telemetry.lateral_error.computed);
    stream << ",\"weighted_lateral_error_m\":";
    AppendJsonNumber(stream, telemetry.lateral_error.weighted_lateral_error_m);
    stream << ",\"weighted_sample_count\":" << telemetry.lateral_error.weighted_sample_count;
    stream << ",\"weight_sum\":";
    AppendJsonNumber(stream, telemetry.lateral_error.weight_sum);
    stream << ",\"reason\":";
    AppendJsonString(stream, telemetry.lateral_error.reason);
    stream << "}";
    stream << ",\"reference_tracking_geometry\":{\"computed\":";
    AppendJsonBool(stream, telemetry.reference_tracking_geometry.computed);
    stream << ",\"lateral_offset_m\":";
    AppendJsonNumber(stream, telemetry.reference_tracking_geometry.lateral_offset_m);
    stream << ",\"heading_error_rad\":";
    AppendJsonNumber(stream, telemetry.reference_tracking_geometry.heading_error_rad);
    stream << ",\"curvature_m_inv\":";
    AppendJsonNumber(stream, telemetry.reference_tracking_geometry.curvature_m_inv);
    stream << ",\"sample_count\":" << telemetry.reference_tracking_geometry.sample_count;
    stream << ",\"reason\":";
    AppendJsonString(stream, telemetry.reference_tracking_geometry.reason);
    stream << "}";
    stream << ",\"reference_control\":{\"ready\":";
    AppendJsonBool(stream, telemetry.reference_control.ready);
    stream << ",\"reason\":";
    AppendJsonString(stream, telemetry.reference_control.reason);
    stream << "}";
    stream << ",\"safety_gate\":{\"veto_active\":";
    AppendJsonBool(stream, telemetry.safety_gate.veto_active);
    stream << ",\"reason\":";
    AppendJsonString(stream, telemetry.safety_gate.reason);
    stream << "}";
    stream << ",\"degraded\":";
    stream << "{\"active\":";
    AppendJsonBool(stream, telemetry.degraded.active);
    stream << ",\"reason\":";
    AppendJsonString(stream, telemetry.degraded.reason);
    stream << "}";
    stream << ",\"yaw_control\":{\"valid\":";
    AppendJsonBool(stream, telemetry.yaw_control.valid);
    stream << ",\"reason\":";
    AppendJsonString(stream, telemetry.yaw_control.reason);
    stream << ",\"turn_output_target\":";
    AppendJsonNumber(stream, telemetry.yaw_control.turn_output_target);
    stream << ",\"lateral_term\":";
    AppendJsonNumber(stream, telemetry.yaw_control.lateral_term);
    stream << ",\"heading_term\":";
    AppendJsonNumber(stream, telemetry.yaw_control.heading_term);
    stream << ",\"curvature_term\":";
    AppendJsonNumber(stream, telemetry.yaw_control.curvature_term);
    stream << "}";
    stream << ",\"tuning_mode_enabled\":";
    AppendJsonBool(stream, telemetry.tuning_mode_enabled);
    stream << ",\"turn_suppressed\":";
    AppendJsonBool(stream, telemetry.turn_suppressed);
    stream << ",\"target_speed_override_enabled\":";
    AppendJsonBool(stream, telemetry.target_speed_override_enabled);
    stream << ",\"target_speed_override_value\":";
    if (telemetry.target_speed_override_enabled) {
        AppendJsonNumber(stream, telemetry.target_speed_override_value);
    } else {
        stream << "null";
    }
    stream << ",\"effective_speed_target\":";
    AppendJsonNumber(stream, telemetry.effective_speed_target);
    stream << ",\"left_speed_target\":";
    AppendJsonNumber(stream, telemetry.left_speed_target);
    stream << ",\"right_speed_target\":";
    AppendJsonNumber(stream, telemetry.right_speed_target);
    stream << ",\"left_measured_speed\":";
    AppendJsonNumber(stream, telemetry.left_measured_speed);
    stream << ",\"right_measured_speed\":";
    AppendJsonNumber(stream, telemetry.right_measured_speed);
    stream << ",\"raw_turn_output\":" << telemetry.raw_turn_output;
    stream << ",\"applied_turn_output\":" << telemetry.applied_turn_output;
    stream << ",\"left_drive_pwm_command\":" << telemetry.left_drive_pwm_command;
    stream << ",\"right_drive_pwm_command\":" << telemetry.right_drive_pwm_command;
    stream << ",\"left_brushless_pwm_command\":" << telemetry.left_brushless_pwm_command;
    stream << ",\"right_brushless_pwm_command\":" << telemetry.right_brushless_pwm_command;
    stream << ",\"left_drive_pwm_unconstrained\":";
    AppendJsonNumber(stream, telemetry.left_drive_pwm_unconstrained);
    stream << ",\"right_drive_pwm_unconstrained\":";
    AppendJsonNumber(stream, telemetry.right_drive_pwm_unconstrained);
    stream << ",\"left_drive_pwm_requested\":" << telemetry.left_drive_pwm_requested;
    stream << ",\"right_drive_pwm_requested\":" << telemetry.right_drive_pwm_requested;
    stream << ",\"left_drive_pwm_desired\":" << telemetry.left_drive_pwm_desired;
    stream << ",\"right_drive_pwm_desired\":" << telemetry.right_drive_pwm_desired;
    stream << ",\"left_drive_pwm_step_limited\":";
    AppendJsonBool(stream, telemetry.left_drive_pwm_step_limited);
    stream << ",\"right_drive_pwm_step_limited\":";
    AppendJsonBool(stream, telemetry.right_drive_pwm_step_limited);
    stream << ",\"left_drive_pwm_reverse_suppressed\":";
    AppendJsonBool(stream, telemetry.left_drive_pwm_reverse_suppressed);
    stream << ",\"right_drive_pwm_reverse_suppressed\":";
    AppendJsonBool(stream, telemetry.right_drive_pwm_reverse_suppressed);
    stream << ",\"left_drive_pwm_floor_adjusted\":";
    AppendJsonBool(stream, telemetry.left_drive_pwm_floor_adjusted);
    stream << ",\"right_drive_pwm_floor_adjusted\":";
    AppendJsonBool(stream, telemetry.right_drive_pwm_floor_adjusted);
    stream << ",\"left_pid_error\":";
    AppendJsonNumber(stream, telemetry.left_pid_error);
    stream << ",\"right_pid_error\":";
    AppendJsonNumber(stream, telemetry.right_pid_error);
    stream << ",\"left_pid_integral\":";
    AppendJsonNumber(stream, telemetry.left_pid_integral);
    stream << ",\"right_pid_integral\":";
    AppendJsonNumber(stream, telemetry.right_pid_integral);
    stream << ",\"left_pid_integral_candidate\":";
    AppendJsonNumber(stream, telemetry.left_pid_integral_candidate);
    stream << ",\"right_pid_integral_candidate\":";
    AppendJsonNumber(stream, telemetry.right_pid_integral_candidate);
    stream << ",\"left_pid_anti_windup_active\":";
    AppendJsonBool(stream, telemetry.left_pid_anti_windup_active);
    stream << ",\"right_pid_anti_windup_active\":";
    AppendJsonBool(stream, telemetry.right_pid_anti_windup_active);
    stream << ",\"left_pid_anti_windup_reason\":";
    AppendJsonString(stream, telemetry.left_pid_anti_windup_reason);
    stream << ",\"right_pid_anti_windup_reason\":";
    AppendJsonString(stream, telemetry.right_pid_anti_windup_reason);
    stream << ",\"actuator_apply_outcome\":";
    AppendJsonString(stream, telemetry.actuator_apply_outcome);
    stream << ",\"actuators_armed\":";
    AppendJsonBool(stream, telemetry.actuators_armed);
    stream << ",\"last_confirmed_left_drive_pwm\":" << telemetry.last_confirmed_left_drive_pwm;
    stream << ",\"last_confirmed_right_drive_pwm\":" << telemetry.last_confirmed_right_drive_pwm;
    stream << ",\"last_confirmed_left_brushless_pwm\":" << telemetry.last_confirmed_left_brushless_pwm;
    stream << ",\"last_confirmed_right_brushless_pwm\":" << telemetry.last_confirmed_right_brushless_pwm;
    stream << '}';
    return stream.str();
}

/// @brief 将助手命令类型枚举值转换为可读字符串
/// @param type 命令类型枚举
/// @return 命令名称字符串
const char* ToString(AssistantCommandType type) {
    switch (type) {
        case AssistantCommandType::kStart:
            return "start";
        case AssistantCommandType::kStop:
            return "stop";
        case AssistantCommandType::kEnableTuningMode:
            return "enable_tuning_mode";
        case AssistantCommandType::kDisableTuningMode:
            return "disable_tuning_mode";
        case AssistantCommandType::kSetTurnSuppressed:
            return "set_turn_suppressed";
        case AssistantCommandType::kSetTargetSpeed:
            return "set_target_speed";
    }
    return "unknown";
}

}  // namespace ls2k::transport
