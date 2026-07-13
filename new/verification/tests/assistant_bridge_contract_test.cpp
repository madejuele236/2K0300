#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "platform/true_ls2k0300/assistant_bridge.hpp"

namespace bridge = ls2k::platform::true_ls2k0300;

namespace {

using Clock = std::chrono::steady_clock;

constexpr auto kConnectTimeout = std::chrono::seconds(3);
constexpr auto kIoTimeout = std::chrono::seconds(2);
constexpr auto kBackoffTransitionTimeout = std::chrono::seconds(1);
bool g_force_send_would_block = false;
}

extern "C" ssize_t __real_send(int socket_fd, const void* buffer, size_t length, int flags);
extern "C" ssize_t __wrap_send(int socket_fd, const void* buffer, size_t length, int flags) {
    if (g_force_send_would_block) {
        g_force_send_would_block = false;
        errno = EAGAIN;
        return -1;
    }
    return __real_send(socket_fd, buffer, length, flags);
}

namespace {

void Expect(bool condition, const char* message) {
    if (!condition) {
        std::cerr << message << '\n';
        std::exit(1);
    }
}

int MakeListener(std::uint16_t& port) {
    const int fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    Expect(fd >= 0, "socket failed");
    const int reuse = 1;
    Expect(setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) == 0,
           "SO_REUSEADDR failed");
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = 0;
    Expect(bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0, "bind failed");
    Expect(listen(fd, 2) == 0, "listen failed");
    const int flags = fcntl(fd, F_GETFL, 0);
    Expect(flags >= 0 && fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0,
           "listener non-blocking configuration failed");
    socklen_t length = sizeof(address);
    Expect(getsockname(fd, reinterpret_cast<sockaddr*>(&address), &length) == 0,
           "getsockname failed");
    port = ntohs(address.sin_port);
    return fd;
}

int AcceptConnected(int listener) {
    const auto deadline = Clock::now() + kConnectTimeout;
    while (Clock::now() < deadline) {
        (void)bridge::PollAssistantBridge();
        const int accepted = accept4(listener, nullptr, nullptr, SOCK_NONBLOCK);
        if (accepted >= 0) {
            while (Clock::now() < deadline) {
                if (bridge::PollAssistantBridge().state == bridge::AssistantBridgeState::kReady) {
                    return accepted;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
            close(accepted);
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    Expect(false, "assistant bridge did not reach ready");
    return -1;
}

std::string PollReceived(std::size_t expected_size) {
    std::string received;
    const auto deadline = Clock::now() + kIoTimeout;
    while (received.size() < expected_size && Clock::now() < deadline) {
        bridge::AssistantBridgePollResult result = bridge::PollAssistantBridge();
        received += result.received_bytes;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return received;
}

void WaitForBackoff() {
    const auto deadline = Clock::now() + kBackoffTransitionTimeout;
    while (Clock::now() < deadline) {
        if (bridge::PollAssistantBridge().state == bridge::AssistantBridgeState::kBackoff) {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    Expect(false, "peer close did not transition to backoff");
}

}  // namespace

int main() {
    std::string detail;
    Expect(bridge::InitializeAssistantBridge({"256.256.256.256", 12345}, detail),
           "invalid DNS fixture was rejected before resolution");
    const bridge::AssistantBridgePollResult dns_failure = bridge::PollAssistantBridge();
    Expect(dns_failure.state == bridge::AssistantBridgeState::kBackoff,
           "DNS failure did not transition to backoff");
    Expect(dns_failure.detail.find("resolve failed") != std::string::npos,
           "DNS failure detail lost the owning resolution error");

    std::uint16_t port = 0;
    const int listener = MakeListener(port);
    Expect(bridge::InitializeAssistantBridge({"127.0.0.1", static_cast<int>(port)}, detail),
           "bridge configuration failed");
    const bridge::AssistantBridgeState initial_connect_state =
        bridge::PollAssistantBridge().state;
    Expect(initial_connect_state == bridge::AssistantBridgeState::kConnecting ||
               initial_connect_state == bridge::AssistantBridgeState::kReady,
           "first non-blocking connect poll must report connecting or ready");

    int peer = AcceptConnected(listener);
    const std::string first = "partial-";
    const std::string second = "input";
    Expect(send(peer, first.data(), first.size(), 0) == static_cast<ssize_t>(first.size()),
           "first partial send failed");
    Expect(PollReceived(first.size()) == first, "first received byte chunk changed");
    Expect(send(peer, second.data(), second.size(), 0) == static_cast<ssize_t>(second.size()),
           "second partial send failed");
    Expect(PollReceived(second.size()) == second, "second received byte chunk changed");

    close(peer);
    WaitForBackoff();

    peer = AcceptConnected(listener);
    const std::string droppable = "best-effort\n";
    g_force_send_would_block = true;
    Expect(!bridge::SendAssistantBytes(
               reinterpret_cast<const std::uint8_t*>(droppable.data()),
               droppable.size(),
               false,
               detail),
           "drop-if-busy send must report a dropped frame");
    Expect(detail.find("busy; dropped this frame") != std::string::npos,
           "drop-if-busy detail changed");
    Expect(bridge::AssistantBridgeReady(),
           "drop-if-busy must not tear down a healthy connection");

    const std::string outbound = "{\"golden\":true}\n";
    Expect(bridge::SendAssistantBytes(reinterpret_cast<const std::uint8_t*>(outbound.data()),
                                      outbound.size(),
                                      true,
                                      detail),
           "reliable send failed");
    std::string observed(outbound.size(), '\0');
    std::size_t observed_size = 0;
    const auto read_deadline = Clock::now() + kIoTimeout;
    while (observed_size < observed.size() && Clock::now() < read_deadline) {
        const ssize_t count = recv(peer,
                                   observed.data() + observed_size,
                                   observed.size() - observed_size,
                                   0);
        if (count > 0) {
            observed_size += static_cast<std::size_t>(count);
        } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    Expect(observed_size == outbound.size() && observed == outbound,
           "reliable output bytes changed");

    const int small_buffer = 4096;
    Expect(setsockopt(peer, SOL_SOCKET, SO_RCVBUF, &small_buffer, sizeof(small_buffer)) == 0,
           "SO_RCVBUF failed");
    const std::vector<std::uint8_t> blocked_payload(16U * 1024U * 1024U, 0x5aU);
    const bool blocked_send = bridge::SendAssistantBytes(blocked_payload.data(),
                                                         blocked_payload.size(),
                                                         true,
                                                         detail);
    Expect(!blocked_send, "backpressured reliable send unexpectedly completed");
    Expect(detail.find("partial frame send") != std::string::npos ||
               detail.find("stayed busy") != std::string::npos,
           "backpressure detail did not identify would-block/partial I/O");
    Expect(!bridge::AssistantBridgeReady(),
           "stalled partial reliable frame must leave ready state");

    close(peer);
    close(listener);
    std::cout << "assistant_bridge_contract_test passed\n";
    return 0;
}
