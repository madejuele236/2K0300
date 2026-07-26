#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
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

void ExpectNear(double actual, double expected, const std::string& message) {
    if (std::abs(actual - expected) > 1.0e-9) {
        throw std::runtime_error(message);
    }
}

class NullDiagnostics final : public ls2k::port::DiagnosticSink {
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
        return disable_ok;
    }
    bool Shutdown(ls2k::port::DiagnosticSink&) override { return true; }
    bool Ready() const override { return true; }

    ls2k::port::ActuatorCommand last_applied_command{};
    int apply_count = 0;
    int disable_count = 0;
    bool disable_ok = true;
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

    void Fail() {
        Require(running_ && static_cast<bool>(on_failure_), "timer failure callback must be active");
        running_ = false;
        on_failure_();
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
    params.motion_stop_ms = 0;
    params.motion_stop_encoder_threshold = 0;
    params.motion_fault_rearm_hold_ms = 0;
    params.reference_time_alignment.enabled = false;
    params.control_snapshot_emit_interval_ms = 1000000;
    params.left_wheel_pid = {0.0, 1.0, 0.0, 1000.0, 1.0};
    params.right_wheel_pid = params.left_wheel_pid;
    return params;
}

void PublishReadyPerception(ls2k::runtime::RuntimeState& state) {
    const std::uint64_t now_ms = ls2k::port::NowMs();
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    state.perception.published = true;
    state.perception.fresh = true;
    state.perception.frame_id += 1;
    state.perception.capture_time_ms = now_ms;
    state.perception.publish_time_ms = now_ms;
    state.perception.reference_capture_time_ms = now_ms;
    state.perception.perception_health.projector_ok = true;
    state.perception.reference_control.ready = true;
    state.perception.reference_tracking_geometry.computed = true;
    state.perception.reference_tracking_geometry.reason = "ok";
}

void SetRunning(ls2k::runtime::RuntimeState& state) {
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    state.motion_state.phase = ls2k::control::MotionPhase::kRunning;
    state.motion_state.phase_entry_ms = ls2k::port::NowMs();
    state.motion_state.last_effective_speed_target = 1.0;
    state.motion_state.last_shaped_command_zero = false;
    state.motion_intent.start_requested = true;
    state.motion_intent.stop_requested = false;
}

void SetStoppingAwaitingCurrentZero(ls2k::runtime::RuntimeState& state) {
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    state.motion_state.phase = ls2k::control::MotionPhase::kStopping;
    state.motion_state.phase_entry_ms = 0;
    state.motion_state.stop_entry_speed_target = 1.0;
    state.motion_state.last_effective_speed_target = 0.0;
    state.motion_state.last_shaped_command_zero = false;
    state.motion_intent.start_requested = false;
    state.motion_intent.stop_requested = true;
}

void SetArithmeticOverflowGeometry(ls2k::runtime::RuntimeState& state) {
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    state.perception.reference_tracking_geometry.computed = true;
    state.perception.reference_tracking_geometry.lateral_offset_m =
        std::numeric_limits<float>::max();
    state.perception.reference_tracking_geometry.reason = "ok";
}

ls2k::observability::ControlDebugSnapshot Snapshot(
    ls2k::runtime::RuntimeState& state) {
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    return state.control_debug_snapshot;
}

ls2k::safety::ControlCycleObservation Observation(
    ls2k::runtime::RuntimeState& state) {
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    return state.control_observation;
}

ls2k::port::ControlCommandHistorySample LatestCommandHistory(
    ls2k::runtime::RuntimeState& state) {
    std::lock_guard<std::mutex> lock(state.shared_mutex);
    Require(state.command_history.count > 0, "command history must contain the current cycle");
    return state.command_history.NewestOffset(0);
}

void TestObservationUsesDeviceAppliedCommand() {
    ls2k::safety::ControlCycleInputs inputs{};
    inputs.command = {-300, 200, 0, 0, false};
    inputs.motion_phase = ls2k::control::MotionPhase::kRunning;
    inputs.apply_ok = true;
    inputs.previous_confirmed_command = {100, 200, 0, 0, false};
    inputs.applied_command = {0, 200, 0, 0, false};
    inputs.previously_armed = true;

    const auto observation = ls2k::safety::ObserveControlCycle(inputs);
    Require(observation.requested_nonzero_output &&
                observation.apply_outcome ==
                    ls2k::safety::ControlApplyOutcome::kDriveCommandApplied,
            "direction dead cycle must retain the nonzero request disposition");
    Require(observation.applied_left_drive_pwm == 0 &&
                observation.applied_right_drive_pwm == 200,
            "last-confirmed PWM must use the device result rather than the request");
}

void TestStoppingSecondEvaluatePublishesPostResetPidFacts() {
    ls2k::port::PlatformBundle platform{};
    platform.imu = std::make_unique<ValidImu>();
    platform.encoder = std::make_unique<QuietEncoder>();
    auto actuator = std::make_unique<RecordingActuator>();
    RecordingActuator* actuator_ptr = actuator.get();
    platform.actuator = std::move(actuator);
    auto timer = std::make_unique<CapturingTimer>();
    CapturingTimer* timer_ptr = timer.get();
    platform.timer = std::move(timer);

    const auto profile = DirectProfile();
    const auto params = TestParams();
    ls2k::runtime::RuntimeState state{};
    state.startup_complete = true;
    NullDiagnostics diagnostics{};
    ls2k::runtime::ControlLoop loop(platform, profile, state, diagnostics);
    Require(loop.Start(params), "control loop must start with fake direct adapters");

    PublishReadyPerception(state);
    SetRunning(state);
    timer_ptr->Fire();
    const auto running = Snapshot(state);
    Require(running.motion_phase == ls2k::control::MotionPhase::kRunning,
            "first cycle must exercise RUNNING");
    ExpectNear(running.left_pid_integral, 1.0, "RUNNING left committed integral");
    ExpectNear(running.right_pid_integral, 1.0, "RUNNING right committed integral");
    ExpectNear(running.left_pid_integral_candidate, 1.0, "RUNNING left candidate integral");
    Require(!running.left_pid_anti_windup_active &&
                running.left_pid_anti_windup_reason ==
                    ls2k::control::WheelPidAntiWindupReason::kNone,
            "normal RUNNING commit facts must remain unchanged");
    Require(actuator_ptr->last_applied_command.left_drive_pwm == 1,
            "RUNNING cycle must apply the integral-controller output");
    Require(actuator_ptr->apply_count == 1 && actuator_ptr->disable_count == 0,
            "RUNNING must use ordinary Apply");

    // Build a real previous applied command of 2 so STOPPING needs one incomplete
    // step before its following cycle can reach zero.
    PublishReadyPerception(state);
    timer_ptr->Fire();
    const auto running_second = Snapshot(state);
    ExpectNear(running_second.left_pid_integral, 2.0,
               "second RUNNING cycle must commit normally");
    Require(actuator_ptr->last_applied_command.left_drive_pwm == 2 &&
                actuator_ptr->apply_count == 2 && actuator_ptr->disable_count == 0,
            "second RUNNING cycle must establish previous PWM 2");

    PublishReadyPerception(state);
    SetStoppingAwaitingCurrentZero(state);
    timer_ptr->Fire();
    const auto stopping = Snapshot(state);
    const auto stopping_observation = Observation(state);
    const auto stopping_history = LatestCommandHistory(state);
    Require(stopping.motion_phase == ls2k::control::MotionPhase::kStopping,
            "first controlled STOPPING step must remain STOPPING");
    Require(actuator_ptr->apply_count == 3 && actuator_ptr->disable_count == 0 &&
                actuator_ptr->last_applied_command.left_drive_pwm == 1,
            "incomplete STOPPING must Apply the one-step PWM ramp");
    Require(stopping_observation.actuators_armed &&
                stopping_observation.apply_outcome ==
                    ls2k::safety::ControlApplyOutcome::kDriveCommandApplied &&
                !stopping_observation.hold_disarmed,
            "incomplete STOPPING observation must remain armed and applied");
    Require(stopping_history.actuator_applied && !stopping_history.hold_disarmed &&
                stopping_history.left_drive_pwm == 1,
            "incomplete STOPPING history must record the applied step");

    PublishReadyPerception(state);
    timer_ptr->Fire();
    const auto stopped = Snapshot(state);
    const auto stopped_observation = Observation(state);
    const auto stopped_history = LatestCommandHistory(state);
    Require(stopped.motion_phase == ls2k::control::MotionPhase::kDisarmed,
            "second STOPPING Evaluate must complete into DISARMED");
    Require(actuator_ptr->apply_count == 3 && actuator_ptr->disable_count == 1,
            "completed STOPPING must Disable without another ordinary Apply");
    Require(actuator_ptr->last_applied_command.left_drive_pwm == 1,
            "Disable must not be misreported as an Apply(0) call");
    Require(!stopped_observation.actuators_armed && stopped_observation.hold_disarmed &&
                stopped_observation.apply_outcome ==
                    ls2k::safety::ControlApplyOutcome::kHeldDisarmedApplied,
            "DISARMED observation must reflect the final hold disposition");
    Require(!stopped_history.actuator_applied && stopped_history.hold_disarmed &&
                stopped_history.left_drive_pwm == 0 && stopped_history.right_drive_pwm == 0,
            "completion history must record Disable disposition and zero command");
    Require(stopped.apply_outcome ==
                ls2k::safety::ControlApplyOutcome::kHeldDisarmedApplied,
            "snapshot apply outcome must match DISARMED observation");
    ExpectNear(stopped.left_pid_integral, 0.0, "post-reset left producer integral");
    ExpectNear(stopped.right_pid_integral, 0.0, "post-reset right producer integral");
    ExpectNear(stopped.left_pid_integral_candidate, 0.0,
               "STOPPING without PID Evaluate must publish a zero candidate fact");
    Require(!stopped.left_pid_anti_windup_active &&
                stopped.left_pid_anti_windup_reason ==
                    ls2k::control::WheelPidAntiWindupReason::kNone,
            "post-reset snapshot must not retain stale anti-windup facts");

    // A subsequent real RUNNING Evaluate must start from the reset controller's zero
    // integral. A stale controller would publish 2.0 here instead of 1.0.
    PublishReadyPerception(state);
    SetRunning(state);
    timer_ptr->Fire();
    const auto rerun = Snapshot(state);
    ExpectNear(rerun.left_pid_integral, 1.0, "left controller state after reset");
    ExpectNear(rerun.right_pid_integral, 1.0, "right controller state after reset");
    Require(!rerun.left_pid_anti_windup_active,
            "normal RUNNING commit after STOPPING reset must remain enabled");
    Require(actuator_ptr->apply_count == 4 && actuator_ptr->disable_count == 1 &&
                actuator_ptr->last_applied_command.left_drive_pwm == 1,
            "RUNNING after reset must return to ordinary Apply");

    (void)loop.Stop();
}

void TestYawArithmeticFailureLatchesFailSafeBeforeWheelMixing() {
    ls2k::port::PlatformBundle platform{};
    platform.imu = std::make_unique<ValidImu>();
    platform.encoder = std::make_unique<QuietEncoder>();
    auto actuator = std::make_unique<RecordingActuator>();
    RecordingActuator* actuator_ptr = actuator.get();
    platform.actuator = std::move(actuator);
    auto timer = std::make_unique<CapturingTimer>();
    CapturingTimer* timer_ptr = timer.get();
    platform.timer = std::move(timer);

    const auto profile = DirectProfile();
    auto params = TestParams();
    params.bev_control_model.lateral_offset_to_wheel_delta_gain = 1000.0;
    ls2k::runtime::RuntimeState state{};
    state.startup_complete = true;
    NullDiagnostics diagnostics{};
    ls2k::runtime::ControlLoop loop(platform, profile, state, diagnostics);
    Require(loop.Start(params), "finite programmatic parameters must start the control loop");

    PublishReadyPerception(state);
    SetArithmeticOverflowGeometry(state);
    SetRunning(state);
    timer_ptr->Fire();

    const auto snapshot = Snapshot(state);
    const auto observation = Observation(state);
    const auto history = LatestCommandHistory(state);
    Require(snapshot.motion_phase == ls2k::control::MotionPhase::kFailSafeLatched,
            "invalid yaw arithmetic must enter the existing fail-safe lifecycle owner");
    Require(snapshot.veto_active &&
                snapshot.veto_reason == ls2k::safety::ControlVetoReason::kYawControlInvalid,
            "final snapshot gate must expose yaw_control_invalid");
    Require(!snapshot.steering.yaw_control.valid &&
                snapshot.steering.yaw_control.reason == "nonfinite_arithmetic",
            "yaw snapshot must expose the first invalid arithmetic fact");
    Require(snapshot.raw_turn_output == 0 && snapshot.applied_turn_output == 0 &&
                snapshot.left_speed_target == 0.0 && snapshot.right_speed_target == 0.0,
            "invalid yaw must not publish an ordinary mixer input or wheel target");
    Require(observation.apply_outcome ==
                ls2k::safety::ControlApplyOutcome::kEmergencyStopApplied &&
                !observation.actuators_armed,
            "invalid yaw must apply the existing emergency-stop command");
    Require(actuator_ptr->apply_count == 1 &&
                actuator_ptr->last_applied_command.emergency_stop,
            "actuator adapter must receive exactly one emergency-stop command");
    Require(history.emergency_stop && !history.actuator_applied &&
                history.applied_turn_output == 0,
            "command history must record fail-closed yaw disposition");
    Require(diagnostics.SawCode("control.yaw.invalid") &&
                diagnostics.SawCode("control.veto.yaw_control_invalid"),
            "runtime diagnostics must expose both yaw failure and final veto owner");

    PublishReadyPerception(state);
    timer_ptr->Fire();
    const auto latched_snapshot = Snapshot(state);
    Require(latched_snapshot.motion_phase == ls2k::control::MotionPhase::kFailSafeLatched,
            "clean current gate must not implicitly rearm the fail-safe lifecycle");
    Require(!latched_snapshot.veto_active &&
                latched_snapshot.veto_reason == ls2k::safety::ControlVetoReason::kNone &&
                !latched_snapshot.steering.safety_gate.veto_active &&
                latched_snapshot.steering.safety_gate.reason == "none",
            "the following tick must expose the current clean gate instead of replaying the yaw veto");
    Require(!latched_snapshot.steering.yaw_control.valid &&
                latched_snapshot.steering.yaw_control.reason == "nonfinite_arithmetic",
            "the first yaw fault must survive later not-computed ticks while its latch is active");

    {
        std::lock_guard<std::mutex> lock(state.shared_mutex);
        state.motion_intent.reset_fault_requested = true;
    }
    PublishReadyPerception(state);
    timer_ptr->Fire();
    const auto reset_snapshot = Snapshot(state);
    Require(reset_snapshot.motion_phase == ls2k::control::MotionPhase::kDisarmed &&
                !reset_snapshot.steering.yaw_control.valid &&
                reset_snapshot.steering.yaw_control.reason == "not_computed",
            "explicit lifecycle reset must clear the yaw latch origin");
    (void)loop.Stop();
}

void TestFinalDisarmedDisableFailurePropagatesWithoutSafeApplyClaim() {
    ls2k::port::PlatformBundle platform{};
    platform.imu = std::make_unique<ValidImu>();
    platform.encoder = std::make_unique<QuietEncoder>();
    auto actuator = std::make_unique<RecordingActuator>();
    RecordingActuator* actuator_ptr = actuator.get();
    platform.actuator = std::move(actuator);
    auto timer = std::make_unique<CapturingTimer>();
    CapturingTimer* timer_ptr = timer.get();
    platform.timer = std::move(timer);

    const auto profile = DirectProfile();
    const auto params = TestParams();
    ls2k::runtime::RuntimeState state{};
    state.startup_complete = true;
    NullDiagnostics diagnostics{};
    ls2k::runtime::ControlLoop loop(platform, profile, state, diagnostics);
    Require(loop.Start(params), "control loop must start for disable-failure injection");

    PublishReadyPerception(state);
    SetRunning(state);
    timer_ptr->Fire();
    PublishReadyPerception(state);
    timer_ptr->Fire();
    Require(actuator_ptr->apply_count == 2,
            "setup must establish a two-step applied command before STOPPING");

    PublishReadyPerception(state);
    SetStoppingAwaitingCurrentZero(state);
    timer_ptr->Fire();
    Require(actuator_ptr->apply_count == 3 && actuator_ptr->disable_count == 0,
            "incomplete STOPPING must still use ordinary Apply");

    actuator_ptr->disable_ok = false;
    PublishReadyPerception(state);
    timer_ptr->Fire();

    const auto snapshot = Snapshot(state);
    const auto observation = Observation(state);
    const auto history = LatestCommandHistory(state);
    Require(actuator_ptr->apply_count == 3 && actuator_ptr->disable_count == 1,
            "final DISARMED cycle must issue exactly one failing Disable and no Apply");
    Require(snapshot.motion_phase == ls2k::control::MotionPhase::kDisarmed &&
                snapshot.apply_outcome == ls2k::safety::ControlApplyOutcome::kApplyFailed,
            "snapshot must preserve final lifecycle decision while reporting disable failure");
    Require(observation.hold_disarmed && observation.actuators_armed &&
                observation.apply_outcome == ls2k::safety::ControlApplyOutcome::kApplyFailed,
            "observation must retain the last confirmed armed state when Disable failed");
    Require(observation.applied_left_drive_pwm == 1 &&
                observation.applied_right_drive_pwm == 1,
            "observation must retain the last confirmed STOPPING PWM when Disable failed");
    Require(snapshot.actuators_armed && snapshot.last_confirmed_left_drive_pwm == 1 &&
                snapshot.last_confirmed_right_drive_pwm == 1,
            "snapshot must separate the failed zero request from the last confirmed PWM");
    Require(!history.actuator_request_succeeded && !history.actuator_applied &&
                history.hold_disarmed && history.left_drive_pwm == 0 &&
                history.right_drive_pwm == 0 && history.actuators_armed &&
                history.last_confirmed_left_drive_pwm == 1 &&
                history.last_confirmed_right_drive_pwm == 1,
            "history must explicitly retain the failed Disable result and zero request");
    {
        std::lock_guard<std::mutex> lock(state.shared_mutex);
        Require(state.actuators_armed && state.last_command.left_drive_pwm == 1 &&
                    state.last_command.right_drive_pwm == 1,
                "runtime state must retain the last confirmed applied command after failed Disable");
    }

    actuator_ptr->disable_ok = true;
    (void)loop.Stop();
}

void TestNormalStopDisableFailurePreservesTerminalEvidence() {
    ls2k::port::PlatformBundle platform{};
    platform.imu = std::make_unique<ValidImu>();
    platform.encoder = std::make_unique<QuietEncoder>();
    auto actuator = std::make_unique<RecordingActuator>();
    RecordingActuator* actuator_ptr = actuator.get();
    platform.actuator = std::move(actuator);
    auto timer = std::make_unique<CapturingTimer>();
    CapturingTimer* timer_ptr = timer.get();
    platform.timer = std::move(timer);
    ls2k::runtime::RuntimeState state{};
    state.startup_complete = true;
    NullDiagnostics diagnostics{};
    const auto profile = DirectProfile();
    ls2k::runtime::ControlLoop loop(platform, profile, state, diagnostics);
    Require(loop.Start(TestParams()), "control loop must start for normal Stop failure test");
    PublishReadyPerception(state);
    SetRunning(state);
    timer_ptr->Fire();
    const int last_applied_pwm = actuator_ptr->last_applied_command.left_drive_pwm;
    Require(last_applied_pwm != 0, "setup must establish a last confirmed drive command");

    actuator_ptr->disable_ok = false;
    const ls2k::runtime::ControlLoopTerminalResult result = loop.Stop();
    Require(!result.ok() && !result.timer_failure && !result.actuator_disable_ok,
            "normal Stop must return its failed Disable result");
    const auto snapshot = Snapshot(state);
    const auto observation = Observation(state);
    const auto history = LatestCommandHistory(state);
    Require(snapshot.motion_phase == ls2k::control::MotionPhase::kDisarmed &&
                snapshot.apply_outcome == ls2k::safety::ControlApplyOutcome::kApplyFailed &&
                observation.apply_outcome == ls2k::safety::ControlApplyOutcome::kApplyFailed,
            "normal Stop failure must publish a consistent DISARMED terminal failure");
    Require(observation.actuators_armed && state.actuators_armed &&
                state.last_command.left_drive_pwm == last_applied_pwm &&
                observation.applied_left_drive_pwm == last_applied_pwm &&
                snapshot.actuators_armed &&
                snapshot.last_confirmed_left_drive_pwm == last_applied_pwm,
            "failed Stop must retain the last confirmed armed state and command");
    Require(!history.actuator_request_succeeded && !history.actuator_applied &&
                history.actuators_armed &&
                history.last_confirmed_left_drive_pwm == last_applied_pwm,
            "normal Stop history must retain the failed zero-output request");

    actuator_ptr->disable_ok = true;
    const ls2k::runtime::ControlLoopTerminalResult repeated = loop.Stop();
    Require(!repeated.ok() && !repeated.actuator_disable_ok &&
                Observation(state).apply_outcome ==
                    ls2k::safety::ControlApplyOutcome::kApplyFailed,
            "later Stop calls must not wash out the prior failure or terminal evidence");
}

void TestTimerFailureDisableFailureReachesPersistentTerminalResult() {
    ls2k::port::PlatformBundle platform{};
    platform.imu = std::make_unique<ValidImu>();
    platform.encoder = std::make_unique<QuietEncoder>();
    auto actuator = std::make_unique<RecordingActuator>();
    RecordingActuator* actuator_ptr = actuator.get();
    platform.actuator = std::move(actuator);
    auto timer = std::make_unique<CapturingTimer>();
    CapturingTimer* timer_ptr = timer.get();
    platform.timer = std::move(timer);
    ls2k::runtime::RuntimeState state{};
    state.startup_complete = true;
    NullDiagnostics diagnostics{};
    const auto profile = DirectProfile();
    ls2k::runtime::ControlLoop loop(platform, profile, state, diagnostics);
    Require(loop.Start(TestParams()), "control loop must start for timer failure test");
    PublishReadyPerception(state);
    SetRunning(state);
    timer_ptr->Fire();
    const int last_applied_pwm = actuator_ptr->last_applied_command.left_drive_pwm;
    actuator_ptr->disable_ok = false;

    timer_ptr->Fail();
    const ls2k::runtime::ControlLoopTerminalResult result = loop.Stop();
    Require(!result.ok() && result.timer_failure && !result.actuator_disable_ok,
            "timer callback result must remain readable by the main-thread Stop call");
    const auto snapshot = Snapshot(state);
    const auto observation = Observation(state);
    const auto history = LatestCommandHistory(state);
    Require(snapshot.motion_phase == ls2k::control::MotionPhase::kFailSafeLatched &&
                snapshot.apply_outcome == ls2k::safety::ControlApplyOutcome::kApplyFailed &&
                observation.motion_phase == ls2k::control::MotionPhase::kFailSafeLatched &&
                observation.apply_outcome == ls2k::safety::ControlApplyOutcome::kApplyFailed,
            "timer Disable failure must preserve FAIL_SAFE_LATCHED structured evidence");
    Require(state.actuators_armed && observation.actuators_armed &&
                state.last_command.left_drive_pwm == last_applied_pwm &&
                observation.applied_left_drive_pwm == last_applied_pwm &&
                snapshot.actuators_armed &&
                snapshot.last_confirmed_left_drive_pwm == last_applied_pwm &&
                !history.actuator_request_succeeded && history.actuators_armed &&
                history.last_confirmed_left_drive_pwm == last_applied_pwm,
            "timer failure must retain last confirmed hardware state and failed Disable history");
    Require(state.exit_requested.load() && state.stop_requested.load() &&
                diagnostics.SawCode("control.timer.disable_failed"),
            "timer failure must request main exit and diagnose the failed Disable");
}

}  // namespace

int main() {
    try {
        TestObservationUsesDeviceAppliedCommand();
        TestStoppingSecondEvaluatePublishesPostResetPidFacts();
        TestYawArithmeticFailureLatchesFailSafeBeforeWheelMixing();
        TestFinalDisarmedDisableFailurePropagatesWithoutSafeApplyClaim();
        TestNormalStopDisableFailurePreservesTerminalEvidence();
        TestTimerFailureDisableFailureReachesPersistentTerminalResult();
    } catch (const std::exception& error) {
        std::cerr << "control_loop_stopping_pid_snapshot_test failed: " << error.what() << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "control_loop_stopping_pid_snapshot_test passed\n";
    return EXIT_SUCCESS;
}
