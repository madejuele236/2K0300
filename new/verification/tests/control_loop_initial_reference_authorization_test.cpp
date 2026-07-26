#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "port/diagnostics.hpp"
#include "port/perf_counter.hpp"
#include "runtime/loops/control_loop.hpp"

namespace {

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class RecordingDiagnostics final : public ls2k::port::DiagnosticSink {
public:
    void Emit(const ls2k::port::DiagnosticEvent& event) override { events.push_back(event); }

    bool SawCode(const std::string& code) const {
        for (const auto& event : events) {
            if (event.code == code) {
                return true;
            }
        }
        return false;
    }

    std::vector<ls2k::port::DiagnosticEvent> events{};
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

class RecordingActuator final : public ls2k::port::IActuatorAdapter {
public:
    bool Initialize(const ls2k::port::HardwareProfile&, ls2k::port::DiagnosticSink&) override {
        return true;
    }
    ls2k::port::ActuatorApplyResult Apply(const ls2k::port::ActuatorCommand& command,
                                          ls2k::port::DiagnosticSink&) override {
        last_applied_command = command;
        ++apply_count;
        return {true, command};
    }
    bool Disable(ls2k::port::DiagnosticSink&) override {
        ++disable_count;
        return true;
    }
    bool Shutdown(ls2k::port::DiagnosticSink&) override { return true; }
    bool Ready() const override { return true; }

    ls2k::port::ActuatorCommand last_applied_command{};
    int apply_count = 0;
    int disable_count = 0;
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
        Require(running_ && static_cast<bool>(callback_), "timer callback must be active");
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
    params.running_speed_target = 1.0;
    params.pwm_floor = 0;
    params.prohibit_reverse_pwm = false;
    params.drive_pwm_step_limit = 1;
    params.motion_unveto_confirm_cycles = 1;
    params.motion_spinup_ms = 0;
    params.motion_stop_ms = 0;
    params.motion_stop_encoder_threshold = 0;
    params.motion_fault_rearm_hold_ms = 0;
    params.reference_time_alignment.enabled = false;
    params.control_snapshot_emit_interval_ms = 1000000;
    params.left_wheel_pid = {0.0, 1.0, 0.0, 1000.0, 1.0};
    params.right_wheel_pid = params.left_wheel_pid;
    return params;
}

void PublishCurrentReference(ls2k::runtime::RuntimeState& state) {
    const std::uint64_t now_ms = ls2k::port::NowMs();
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    state.perception.published = true;
    state.perception.fresh = true;
    state.perception.frame_id += 1;
    state.perception.capture_time_ms = now_ms;
    state.perception.publish_time_ms = now_ms;
    state.perception.reference_capture_time_ms = now_ms;
    state.perception.perception_health.projector_ok = true;
    state.perception.visual_reference_selection.present = true;
    state.perception.visual_reference_selection.reason = "selected";
    state.perception.reference_mode = "interval_center";
    state.perception.reference_path.mode = ls2k::port::ReferenceMode::kIntervalCenter;
    state.perception.reference_control.ready = true;
    state.perception.reference_control.degraded = false;
    state.perception.reference_control.reason = "ok";
    state.perception.reference_tracking_geometry.computed = true;
    state.perception.reference_tracking_geometry.reason = "ok";
}

void PublishHoldOnlyReference(ls2k::runtime::RuntimeState& state) {
    const std::uint64_t now_ms = ls2k::port::NowMs();
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    state.perception.published = true;
    state.perception.fresh = true;
    state.perception.frame_id += 1;
    state.perception.capture_time_ms = now_ms;
    state.perception.publish_time_ms = now_ms;
    state.perception.perception_health.projector_ok = true;
    state.perception.visual_reference_selection = {};
    state.perception.reference_mode = "hold_last";
    state.perception.reference_path.mode = ls2k::port::ReferenceMode::kHoldLast;
    state.perception.reference_control.ready = true;
    state.perception.reference_control.degraded = true;
    state.perception.reference_control.reason = "reference_hold";
    state.perception.reference_tracking_geometry.computed = true;
    state.perception.reference_tracking_geometry.reason = "ok";
}

void RequestAutomaticStartEquivalent(ls2k::runtime::RuntimeState& state) {
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    state.motion_intent.start_requested = true;
    state.motion_intent.stop_requested = false;
}

ls2k::observability::ControlDebugSnapshot Snapshot(ls2k::runtime::RuntimeState& state) {
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    return state.control_debug_snapshot;
}

struct ControlFixture {
    ls2k::port::PlatformBundle platform{};
    RecordingActuator* actuator = nullptr;
    CapturingTimer* timer = nullptr;
    ls2k::port::HardwareProfile profile = DirectProfile();
    ls2k::runtime::RuntimeState state{};
    RecordingDiagnostics diagnostics{};
    std::unique_ptr<ls2k::runtime::ControlLoop> loop{};

    ControlFixture() {
        platform.imu = std::make_unique<ValidImu>();
        platform.encoder = std::make_unique<QuietEncoder>();
        auto recording_actuator = std::make_unique<RecordingActuator>();
        actuator = recording_actuator.get();
        platform.actuator = std::move(recording_actuator);
        auto capturing_timer = std::make_unique<CapturingTimer>();
        timer = capturing_timer.get();
        platform.timer = std::move(capturing_timer);
        state.startup_complete = true;
        loop = std::make_unique<ls2k::runtime::ControlLoop>(
            platform, profile, state, diagnostics);
        Require(loop->Start(TestParams()), "control loop must start with fake direct adapters");
    }

    ~ControlFixture() {
        if (loop) {
            (void)loop->Stop();
        }
    }
};

void TestHoldOnlyReferenceCannotAuthorizeInitialStart() {
    ControlFixture fixture{};

    // Establish the historical hold source with a momentary current-frame
    // reference, but do not request motion in that cycle.
    PublishCurrentReference(fixture.state);
    fixture.timer->Fire();
    Require(fixture.actuator->apply_count == 0,
            "current reference without start intent must remain disarmed");

    // The next frame has no current visual selection and exposes only the
    // historical hold. This is the production sequence that previously drove.
    PublishHoldOnlyReference(fixture.state);
    RequestAutomaticStartEquivalent(fixture.state);
    fixture.timer->Fire();
    fixture.timer->Fire();
    fixture.timer->Fire();

    const auto snapshot = Snapshot(fixture.state);
    Require(snapshot.motion_phase == ls2k::control::MotionPhase::kStartRequested,
            "hold-only readiness must keep an initial start request blocked");
    Require(snapshot.veto_active &&
                snapshot.veto_reason ==
                    ls2k::safety::ControlVetoReason::kInitialReferenceHoldNotAllowed,
            "blocked hold-only start must retain its specific gate reason");
    Require(snapshot.left_drive_pwm_requested == 0 && snapshot.right_drive_pwm_requested == 0 &&
                snapshot.left_drive_pwm_command == 0 && snapshot.right_drive_pwm_command == 0 &&
                fixture.actuator->apply_count == 0,
            "hold-only initial start must not request or apply drive output");
    Require(fixture.diagnostics.SawCode("motion.start.blocked") &&
                fixture.diagnostics.SawCode(
                    "control.veto.initial_reference_hold_not_allowed"),
            "hold-only initial start must remain observable at the production boundary");
}

void TestCurrentReferenceStartsAndRunningHoldRemainsAllowed() {
    ControlFixture fixture{};

    PublishCurrentReference(fixture.state);
    RequestAutomaticStartEquivalent(fixture.state);
    fixture.timer->Fire();
    fixture.timer->Fire();
    fixture.timer->Fire();

    const auto running_current = Snapshot(fixture.state);
    Require(running_current.motion_phase == ls2k::control::MotionPhase::kRunning &&
                !running_current.veto_active && fixture.actuator->apply_count > 0 &&
                fixture.actuator->last_applied_command.left_drive_pwm != 0 &&
                fixture.actuator->last_applied_command.right_drive_pwm != 0,
            "a current valid reference must authorize normal startup and nonzero drive");

    const int apply_count_before_hold = fixture.actuator->apply_count;
    PublishHoldOnlyReference(fixture.state);
    fixture.timer->Fire();
    const auto running_hold = Snapshot(fixture.state);
    Require(running_hold.motion_phase == ls2k::control::MotionPhase::kRunning &&
                !running_hold.veto_active &&
                fixture.actuator->apply_count == apply_count_before_hold + 1 &&
                fixture.actuator->last_applied_command.left_drive_pwm != 0 &&
                fixture.actuator->last_applied_command.right_drive_pwm != 0,
            "hold readiness after legal RUNNING entry must retain degraded-run behavior");
}

}  // namespace

int main() {
    try {
        TestHoldOnlyReferenceCannotAuthorizeInitialStart();
        TestCurrentReferenceStartsAndRunningHoldRemainsAllowed();
    } catch (const std::exception& error) {
        std::cerr << "control_loop_initial_reference_authorization_test failed: "
                  << error.what() << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "control_loop_initial_reference_authorization_test passed\n";
    return EXIT_SUCCESS;
}
