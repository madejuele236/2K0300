#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "transport/assistant_protocol.hpp"

namespace {

void Expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

void TestGoldenOutboundFrame() {
    const std::string json = ls2k::transport::EncodeAssistantAck(7, true);
    Expect(json == "{\"type\":\"ack\",\"seq\":7,\"outcome\":\"accepted\"}",
           "ack JSON golden vector changed");
    Expect(ls2k::transport::EncodeAssistantJsonFrame(json) == json + "\n",
           "wire frame must append exactly one LF byte");
}

void TestPartialAndCombinedInboundChunks() {
    ls2k::transport::AssistantProtocolDecoder decoder(2.0);
    const std::string first = "{\"type\":\"command\",\"cmd\":\"start\",\"seq\":41}";
    const std::string second = "{\"type\":\"command\",\"cmd\":\"stop\",\"seq\":42}";

    const auto incomplete = decoder.PushBytes(
        reinterpret_cast<const std::uint8_t*>(first.data()), first.size() - 3U);
    Expect(incomplete.empty(), "partial protocol line must not emit a message");

    const std::string tail_and_second = first.substr(first.size() - 3U) + "\r\n" + second + "\n";
    const auto complete = decoder.PushBytes(
        reinterpret_cast<const std::uint8_t*>(tail_and_second.data()), tail_and_second.size());
    Expect(complete.size() == 2U, "completed and combined lines must emit two messages");
    Expect(complete[0].type == ls2k::transport::AssistantInboundMessageType::kCommand &&
               complete[0].command.type == ls2k::transport::AssistantCommandType::kStart &&
               complete[0].command.seq == 41U,
           "first fragmented command decoded incorrectly");
    Expect(complete[1].type == ls2k::transport::AssistantInboundMessageType::kCommand &&
               complete[1].command.type == ls2k::transport::AssistantCommandType::kStop &&
               complete[1].command.seq == 42U,
           "second combined command decoded incorrectly");
}

void TestOversizeAndResetContract() {
    ls2k::transport::AssistantProtocolDecoder decoder(2.0);
    const std::string oversized(4097U, 'x');
    const auto rejected = decoder.PushBytes(
        reinterpret_cast<const std::uint8_t*>(oversized.data()), oversized.size());
    Expect(rejected.size() == 1U &&
               rejected[0].type == ls2k::transport::AssistantInboundMessageType::kInputRejected &&
               rejected[0].reason == "input line too long",
           "oversized incomplete line must be rejected at the protocol boundary");

    const std::string partial = "{\"type\":\"command\"";
    (void)decoder.PushBytes(reinterpret_cast<const std::uint8_t*>(partial.data()), partial.size());
    decoder.Reset();
    const std::string valid = "{\"type\":\"command\",\"cmd\":\"start\",\"seq\":9}\n";
    const auto after_reset = decoder.PushBytes(
        reinterpret_cast<const std::uint8_t*>(valid.data()), valid.size());
    Expect(after_reset.size() == 1U && after_reset[0].command.seq == 9U,
           "reset must discard bytes from the previous connection");
}

}  // namespace

int main() {
    TestGoldenOutboundFrame();
    TestPartialAndCombinedInboundChunks();
    TestOversizeAndResetContract();
    std::cout << "assistant_protocol_contract_test passed\n";
    return 0;
}
