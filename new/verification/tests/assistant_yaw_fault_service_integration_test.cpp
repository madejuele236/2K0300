#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "platform/true_ls2k0300/assistant_bridge.hpp"
#include "port/diagnostics.hpp"
#include "runtime/loops/control_loop.hpp"
#include "runtime/services/assistant_service.hpp"

namespace {

std::vector<std::string> g_sent_json_lines;

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class NullDiagnostics final : public ls2k::port::DiagnosticSink {
public:
    void Emit(const ls2k::port::DiagnosticEvent&) override {}
};

class ValidImu final : public ls2k::port::IImuAdapter {
public:
    bool Initialize(const ls2k::port::HardwareProfile&, ls2k::port::DiagnosticSink&) override {
        return true;
    }
    ls2k::port::ImuSample Read(ls2k::port::DiagnosticSink&) override {
        ls2k::port::ImuSample sample{};
        sample.valid = true;
        sample.capture_time_ms = ls2k::port::NowMs();
        return sample;
    }
    void Shutdown(ls2k::port::DiagnosticSink&) override {}
    bool Ready() const override { return true; }
};

class QuietEncoder final : public ls2k::port::IEncoderAdapter {
public:
    bool Initialize(const ls2k::port::HardwareProfile&, ls2k::port::DiagnosticSink&) override {
        return true;
    }
    ls2k::port::EncoderDelta ReadDelta(ls2k::port::DiagnosticSink&) override {
        ls2k::port::EncoderDelta sample{};
        sample.valid = true;
        sample.capture_time_ms = ls2k::port::NowMs();
        return sample;
    }
    void Shutdown(ls2k::port::DiagnosticSink&) override {}
    bool Ready() const override { return true; }
};

class SuccessfulActuator final : public ls2k::port::IActuatorAdapter {
public:
    bool Initialize(const ls2k::port::HardwareProfile&, ls2k::port::DiagnosticSink&) override {
        return true;
    }
    ls2k::port::ActuatorApplyResult Apply(const ls2k::port::ActuatorCommand& command,
                                          ls2k::port::DiagnosticSink&) override {
        return {true, command};
    }
    bool Disable(ls2k::port::DiagnosticSink&) override { return true; }
    bool Shutdown(ls2k::port::DiagnosticSink&) override { return true; }
    bool Ready() const override { return true; }
};

class CapturingTimer final : public ls2k::port::ITimerAdapter {
public:
    bool Start(const ls2k::port::SubsystemProfile&,
               std::uint32_t,
               std::function<void()> callback,
               std::function<void()> on_failure,
               ls2k::port::DiagnosticSink&) override {
        callback_ = std::move(callback);
        on_failure_ = std::move(on_failure);
        running_ = true;
        return true;
    }
    void Stop(ls2k::port::DiagnosticSink&) override { running_ = false; }
    bool Running() const override { return running_; }
    void Fire() {
        Require(running_ && static_cast<bool>(callback_), "control timer callback must be active");
        callback_();
    }

private:
    std::function<void()> callback_{};
    std::function<void()> on_failure_{};
    bool running_ = false;
};

ls2k::port::HardwareProfile DirectProfile() {
    ls2k::port::HardwareProfile profile{};
    profile.imu = {ls2k::port::SubsystemMode::kDirectMatch, "test-imu"};
    profile.encoder = {ls2k::port::SubsystemMode::kDirectMatch, "test-encoder"};
    profile.actuator = {ls2k::port::SubsystemMode::kDirectMatch, "test-actuator"};
    profile.timer = {ls2k::port::SubsystemMode::kDirectMatch, "test-timer"};
    return profile;
}

ls2k::port::RuntimeParameters TestParams() {
    ls2k::port::RuntimeParameters params{};
    params.assistant_enabled = true;
    params.assistant_tcp.host = "test-host";
    params.assistant_tcp.port = 12345;
    params.running_speed_target = 1.0;
    params.pwm_floor = 0;
    params.drive_pwm_step_limit = 1;
    params.reference_time_alignment.enabled = false;
    params.control_snapshot_emit_interval_ms = 1000000;
    params.bev_control_model.lateral_offset_to_wheel_delta_gain = 1000.0;
    return params;
}

void PublishOverflowPerception(ls2k::runtime::RuntimeState& state) {
    const std::uint64_t now_ms = ls2k::port::NowMs();
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    state.perception.published = true;
    state.perception.fresh = true;
    ++state.perception.frame_id;
    state.perception.capture_time_ms = now_ms;
    state.perception.publish_time_ms = now_ms;
    state.perception.reference_capture_time_ms = now_ms;
    state.perception.perception_health.projector_ok = true;
    state.perception.reference_control.ready = true;
    state.perception.reference_tracking_geometry.computed = true;
    state.perception.reference_tracking_geometry.lateral_offset_m =
        std::numeric_limits<float>::max();
    state.perception.reference_tracking_geometry.reason = "ok";
}

void SetRunning(ls2k::runtime::RuntimeState& state) {
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    state.motion_state.phase = ls2k::control::MotionPhase::kRunning;
    state.motion_state.phase_entry_ms = ls2k::port::NowMs();
    state.motion_state.last_effective_speed_target = 1.0;
    state.motion_intent.start_requested = true;
}

void WriteArtifact(const char* name, const std::string& json) {
    const char* directory = std::getenv("LS2K_STRICT_JSON_ARTIFACT_DIR");
    Require(directory != nullptr && directory[0] != '\0', "artifact directory must be configured");
    std::ofstream output(std::string(directory) + "/" + name,
                         std::ios::binary | std::ios::trunc);
    Require(output.is_open(), std::string("failed to open artifact ") + name);
    output << json;
    Require(output.good(), std::string("failed to write artifact ") + name);
}

}  // namespace

namespace ls2k::platform::true_ls2k0300 {

bool InitializeAssistantBridge(const AssistantBridgeConfig&, std::string& detail) {
    detail = "test bridge ready";
    return true;
}

AssistantBridgePollResult PollAssistantBridge() {
    return {AssistantBridgeState::kReady, false, "test bridge ready", {}};
}

bool AssistantBridgeReady() {
    return true;
}

bool SendAssistantBytes(const std::uint8_t* data,
                        std::size_t length,
                        bool,
                        std::string& detail) {
    std::string frame(reinterpret_cast<const char*>(data), length);
    if (!frame.empty() && frame.back() == '\n') {
        frame.pop_back();
    }
    g_sent_json_lines.push_back(std::move(frame));
    detail = "captured";
    return true;
}

}  // namespace ls2k::platform::true_ls2k0300

int main() {
    try {
        ls2k::port::PlatformBundle platform{};
        platform.imu = std::make_unique<ValidImu>();
        platform.encoder = std::make_unique<QuietEncoder>();
        platform.actuator = std::make_unique<SuccessfulActuator>();
        auto timer = std::make_unique<CapturingTimer>();
        CapturingTimer* timer_ptr = timer.get();
        platform.timer = std::move(timer);

        const auto profile = DirectProfile();
        const auto params = TestParams();
        ls2k::runtime::RuntimeState state{};
        state.startup_complete = true;
        NullDiagnostics diagnostics{};
        ls2k::runtime::ControlLoop control_loop(platform, profile, state, diagnostics);
        ls2k::runtime::AssistantService assistant_service{};
        Require(control_loop.Start(params), "control loop must start");
        assistant_service.Start(params, diagnostics);

        {
            std::lock_guard<std::mutex> lock(state.shared_mutex);
            state.control_debug_snapshot.valid = true;
            state.control_debug_snapshot.cycle_count = 900;
            state.control_debug_snapshot.motion_phase =
                ls2k::control::MotionPhase::kFailSafeLatched;
        }
        assistant_service.Tick(state, diagnostics);
        Require(g_sent_json_lines.empty(),
                "ordinary non-yaw fail-safe snapshot must remain filtered");

        PublishOverflowPerception(state);
        SetRunning(state);
        timer_ptr->Fire();
        assistant_service.Tick(state, diagnostics);
        Require(g_sent_json_lines.size() == 1,
                "fault tick must publish through the real AssistantService entry");
        Require(g_sent_json_lines[0].find("\"motion_phase\":\"FAIL_SAFE_LATCHED\"") !=
                    std::string::npos &&
                    g_sent_json_lines[0].find(
                        "\"yaw_control\":{\"valid\":false,\"reason\":\"nonfinite_arithmetic\"") !=
                    std::string::npos,
                "fault-tick telemetry must carry the yaw latch origin");

        PublishOverflowPerception(state);
        timer_ptr->Fire();
        std::this_thread::sleep_for(std::chrono::milliseconds(210));
        assistant_service.Tick(state, diagnostics);
        Require(g_sent_json_lines.size() == 2,
                "later fail-safe tick must pass the unchanged service interval and cycle filters");
        Require(g_sent_json_lines[1].find("\"safety_gate\":{\"veto_active\":false,\"reason\":\"none\"") !=
                    std::string::npos &&
                    g_sent_json_lines[1].find(
                        "\"yaw_control\":{\"valid\":false,\"reason\":\"nonfinite_arithmetic\"") !=
                    std::string::npos,
                "later telemetry must keep the first yaw cause while publishing the current clean gate");

        WriteArtifact("assistant_yaw_fault_first.json", g_sent_json_lines[0]);
        WriteArtifact("assistant_yaw_fault_later.json", g_sent_json_lines[1]);
        (void)control_loop.Stop();
    } catch (const std::exception& error) {
        std::cerr << "assistant_yaw_fault_service_integration_test failed: "
                  << error.what() << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "assistant_yaw_fault_service_integration_test passed\n";
    return EXIT_SUCCESS;
}
