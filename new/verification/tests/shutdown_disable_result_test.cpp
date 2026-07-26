#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "runtime/lifecycle/shutdown.hpp"

namespace {

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class CapturingDiagnostics final : public ls2k::port::DiagnosticSink {
public:
    void Emit(const ls2k::port::DiagnosticEvent& event) override { events.push_back(event); }
    bool Saw(const std::string& code, ls2k::port::DiagnosticLevel level) const {
        for (const auto& event : events) {
            if (event.code == code && event.level == level) {
                return true;
            }
        }
        return false;
    }

    std::vector<ls2k::port::DiagnosticEvent> events{};
};

class FailingDisableActuator final : public ls2k::port::IActuatorAdapter {
public:
    bool Initialize(const ls2k::port::HardwareProfile&, ls2k::port::DiagnosticSink&) override {
        return true;
    }
    ls2k::port::ActuatorApplyResult Apply(const ls2k::port::ActuatorCommand& command,
                                          ls2k::port::DiagnosticSink&) override {
        return {true, command};
    }
    bool Disable(ls2k::port::DiagnosticSink&) override {
        ++disable_count;
        return disable_ok;
    }
    bool Shutdown(ls2k::port::DiagnosticSink&) override {
        ++shutdown_count;
        return shutdown_ok;
    }
    bool Ready() const override { return true; }

    int disable_count = 0;
    int shutdown_count = 0;
    bool disable_ok = false;
    bool shutdown_ok = true;
};

}  // namespace

int main() {
    try {
        ls2k::port::PlatformBundle platform{};
        auto actuator = std::make_unique<FailingDisableActuator>();
        FailingDisableActuator* actuator_ptr = actuator.get();
        platform.actuator = std::move(actuator);
        ls2k::runtime::RuntimeState state{};
        state.actuators_armed = true;
        state.last_command.left_drive_pwm = 321;
        state.control_observation.actuators_armed = true;
        CapturingDiagnostics diagnostics{};

        const ls2k::runtime::ShutdownResult result =
            ls2k::runtime::RunShutdown(platform, state, diagnostics);

        Require(actuator_ptr->disable_count == 1 && actuator_ptr->shutdown_count == 1,
                "shutdown must consume one Disable result before releasing the adapter");
        Require(!result.ok() && !result.actuator_disable_ok && result.actuator_shutdown_ok,
                "shutdown result must preserve Disable failure even when Shutdown succeeds later");
        Require(state.actuators_armed && state.stop_requested.load() &&
                    state.exit_requested.load(),
                "failed Disable must retain the last confirmed armed state");
        Require(state.last_command.left_drive_pwm == 321 &&
                    state.control_observation.apply_outcome ==
                        ls2k::safety::ControlApplyOutcome::kApplyFailed &&
                    state.control_debug_snapshot.apply_outcome ==
                        ls2k::safety::ControlApplyOutcome::kApplyFailed &&
                    state.control_debug_snapshot.actuators_armed &&
                    state.control_debug_snapshot.last_confirmed_left_drive_pwm == 321 &&
                    state.command_history.count > 0 &&
                    !state.command_history.NewestOffset(0).actuator_request_succeeded &&
                    state.command_history.NewestOffset(0).actuators_armed &&
                    state.command_history.NewestOffset(0).last_confirmed_left_drive_pwm == 321,
                "shutdown failure must preserve last command and structured terminal evidence");
        Require(diagnostics.Saw("shutdown.actuator.disable_failed",
                                ls2k::port::DiagnosticLevel::kFailSafe),
                "shutdown must explicitly record the failed safe-disable request");
        Require(diagnostics.Saw("shutdown.complete.unconfirmed",
                                ls2k::port::DiagnosticLevel::kFailSafe) &&
                    !diagnostics.Saw("shutdown.complete", ls2k::port::DiagnosticLevel::kInfo),
                "shutdown must not claim that actuators were disabled when Disable failed");

        ls2k::port::PlatformBundle shutdown_fail_platform{};
        auto shutdown_fail_actuator = std::make_unique<FailingDisableActuator>();
        FailingDisableActuator* shutdown_fail_ptr = shutdown_fail_actuator.get();
        shutdown_fail_ptr->disable_ok = true;
        shutdown_fail_ptr->shutdown_ok = false;
        shutdown_fail_platform.actuator = std::move(shutdown_fail_actuator);
        ls2k::runtime::RuntimeState shutdown_fail_state{};
        const ls2k::runtime::ShutdownResult shutdown_fail_result =
            ls2k::runtime::RunShutdown(shutdown_fail_platform,
                                       shutdown_fail_state,
                                       diagnostics);
        Require(shutdown_fail_result.actuator_disable_ok &&
                    !shutdown_fail_result.actuator_shutdown_ok &&
                    !shutdown_fail_result.ok(),
                "actuator Shutdown failure must independently fail RunShutdown");

        ls2k::port::PlatformBundle prior_fail_platform{};
        auto prior_fail_actuator = std::make_unique<FailingDisableActuator>();
        prior_fail_actuator->disable_ok = true;
        prior_fail_actuator->shutdown_ok = true;
        prior_fail_platform.actuator = std::move(prior_fail_actuator);
        ls2k::runtime::RuntimeState prior_fail_state{};
        prior_fail_state.control_observation.apply_outcome =
            ls2k::safety::ControlApplyOutcome::kApplyFailed;
        const ls2k::runtime::ShutdownResult prior_fail_result =
            ls2k::runtime::RunShutdown(prior_fail_platform,
                                       prior_fail_state,
                                       diagnostics);
        Require(prior_fail_result.prior_terminal_failure &&
                    prior_fail_result.actuator_disable_ok &&
                    prior_fail_result.actuator_shutdown_ok &&
                    !prior_fail_result.ok(),
                "later successful shutdown stages must not wash out a prior terminal failure");
    } catch (const std::exception& error) {
        std::cerr << "shutdown_disable_result_test failed: " << error.what() << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "shutdown_disable_result_test passed\n";
    return EXIT_SUCCESS;
}
