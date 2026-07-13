#ifndef LS2K_TRANSPORT_ASSISTANT_LINK_HPP
#define LS2K_TRANSPORT_ASSISTANT_LINK_HPP

#include <memory>
#include <string>
#include <vector>

#include "transport/assistant_protocol.hpp"
#include "port/diagnostics.hpp"
#include "port/runtime_parameter_types.hpp"

namespace ls2k::transport {

/// @brief 助手轮询结果结构体
///
/// 封装一次 Poll() 调用的完整输出，包括连接状态变化和所有已解码的入站消息。
struct AssistantPollResult {
    /// 当前是否已就绪（连接已建立且可通信）
    bool ready = false;
    /// 本次轮询中从"未就绪"变为"就绪"
    bool became_ready = false;
    /// 连接已丢失（从就绪变为未就绪，或挂起断连被确认）
    bool connection_lost = false;
    /// 本轮收到的已解码入站消息列表
    std::vector<AssistantInboundMessage> inbound_messages{};
};

/// @brief JSON 行发送可靠性级别枚举
enum class AssistantJsonSendReliability {
    kBestEffort = 0,   ///< 尽力而为：不保证送达
    kReliable,          ///< 可靠：要求底层确认为止
};

/// @brief 助手通信链接类
///
/// 管理外部助手（远程控制台/监控）的 TCP 连接生命周期：
/// 初始化、轮询接收、发送 JSON 消息。内部通过 AssistantBridge
/// 与实际的 socket/I/O 交互。
class AssistantLink {
public:
    /// @brief 初始化助手链接
    /// @param params 运行时参数（含助手开关、TCP 端点配置）
    /// @param diagnostics 诊断输出接口
    /// @return 初始化是否成功
    bool Initialize(const port::RuntimeParameters& params, port::DiagnosticSink& diagnostics);

    /// @brief 轮询助手链接，检查状态并收取入站消息
    /// @param diagnostics 诊断输出接口
    /// @return 轮询结果（状态变化、入站消息等）
    AssistantPollResult Poll(port::DiagnosticSink& diagnostics);

    /// @brief 发布一条 JSON 行消息
    /// @param line 要发送的 JSON 字符串（不含换行符）
    /// @param reliability 发送可靠性级别
    /// @param diagnostics 诊断输出接口
    /// @return 发送是否成功（kReliable 模式下保证送达）
    bool PublishJsonLine(const std::string& line,
                         AssistantJsonSendReliability reliability,
                         port::DiagnosticSink& diagnostics);

    /// @brief 检查助手链接是否已就绪
    /// @return true 表示连接已建立且可通信
    bool Ready() const;

private:
    /// 是否已配置
    bool configured_ = false;
    /// 当前是否已就绪（连接状态）
    bool ready_ = false;
    /// 是否挂起断连通知（在下次 Poll 中上报）
    bool disconnect_pending_ = false;
    /// 上一次上报的桥接状态码
    int last_state_code_ = -1;
    /// 运行最大目标速度，用于校验助手速度指令上限
    double max_target_speed_ = 0.0;
    /// 项目协议层拥有跨 Poll 的 byte-level JSON 行组帧状态。
    std::unique_ptr<AssistantProtocolDecoder> protocol_decoder_{};
};

}  // namespace ls2k::transport

#endif  // LS2K_TRANSPORT_ASSISTANT_LINK_HPP
