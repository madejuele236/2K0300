#include <cstdlib>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#define main ls2k_production_main_not_used_by_bench_test
#include "../../user/main.cpp"
#undef main

namespace {

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class FixedEncoder final : public ls2k::port::IEncoderAdapter {
public:
    bool Initialize(const ls2k::port::HardwareProfile&, ls2k::port::DiagnosticSink&) override {
        return true;
    }
    ls2k::port::EncoderDelta ReadDelta(ls2k::port::DiagnosticSink&) override {
        ls2k::port::EncoderDelta sample{};
        sample.valid = true;
        return sample;
    }
    void Shutdown(ls2k::port::DiagnosticSink&) override {}
    bool Ready() const override { return true; }
};

class ResultActuator final : public ls2k::port::IActuatorAdapter {
public:
    ResultActuator(bool apply_ok, bool disable_ok)
        : apply_ok_(apply_ok), disable_ok_(disable_ok) {}
    bool Initialize(const ls2k::port::HardwareProfile&, ls2k::port::DiagnosticSink&) override {
        return true;
    }
    ls2k::port::ActuatorApplyResult Apply(const ls2k::port::ActuatorCommand& command,
                                          ls2k::port::DiagnosticSink&) override {
        ++apply_count;
        return {apply_ok_, apply_ok_ ? command : ls2k::port::ActuatorCommand{}};
    }
    bool Disable(ls2k::port::DiagnosticSink&) override {
        ++disable_count;
        return disable_ok_;
    }
    bool Shutdown(ls2k::port::DiagnosticSink&) override { return true; }
    bool Ready() const override { return true; }

    bool apply_ok_ = true;
    bool disable_ok_ = true;
    int apply_count = 0;
    int disable_count = 0;
};

BenchPwmPulseResult RunCase(bool apply_ok,
                            bool disable_ok,
                            std::string& emitted_summary) {
    ls2k::port::PlatformBundle platform{};
    platform.encoder = std::make_unique<FixedEncoder>();
    auto actuator = std::make_unique<ResultActuator>(apply_ok, disable_ok);
    ResultActuator* actuator_ptr = actuator.get();
    platform.actuator = std::move(actuator);
    ls2k::runtime::RuntimeState state{};
    state.low_voltage_last_sample.valid = true;
    state.low_voltage_last_sample.emergency = false;
    state.low_voltage_emergency.store(false);
    ls2k::port::StdoutDiagnostics diagnostics{};

    std::ostringstream captured_out;
    std::ostringstream captured_err;
    std::streambuf* old_out = std::cout.rdbuf(captured_out.rdbuf());
    std::streambuf* old_err = std::cerr.rdbuf(captured_err.rdbuf());
    const BenchPwmPulseResult result = RunBenchPwmPulse(platform, state, diagnostics);
    std::cout.rdbuf(old_out);
    std::cerr.rdbuf(old_err);
    emitted_summary = captured_out.str() + captured_err.str();
    Require(actuator_ptr->apply_count == 1 && actuator_ptr->disable_count == 1,
            "bench must attempt both Apply and final Disable exactly once");
    if (!disable_ok) {
        Require(state.control_observation.apply_outcome ==
                    ls2k::safety::ControlApplyOutcome::kApplyFailed &&
                    state.control_debug_snapshot.apply_outcome ==
                        ls2k::safety::ControlApplyOutcome::kApplyFailed &&
                    state.command_history.count > 0 &&
                    !state.command_history.NewestOffset(0).actuator_request_succeeded,
                "bench Disable failure must publish structured terminal evidence");
        Require(state.actuators_armed == apply_ok &&
                    (!apply_ok || state.last_command.left_drive_pwm == 123) &&
                    state.control_debug_snapshot.actuators_armed == apply_ok &&
                    state.control_debug_snapshot.last_confirmed_left_drive_pwm ==
                        (apply_ok ? 123 : 0) &&
                    state.command_history.NewestOffset(0).actuators_armed == apply_ok &&
                    state.command_history.NewestOffset(0).last_confirmed_left_drive_pwm ==
                        (apply_ok ? 123 : 0),
                "bench Disable failure must retain the last confirmed hardware state");
    }
    return result;
}

void CheckCase(bool apply_ok, bool disable_ok) {
    std::string summary;
    const BenchPwmPulseResult result = RunCase(apply_ok, disable_ok, summary);
    Require(result.requested && result.apply_ok == apply_ok &&
                result.disable_ok == disable_ok &&
                result.ok() == (apply_ok && disable_ok),
            "bench result must preserve independent Apply and Disable facts");
    Require(summary.find(std::string("apply_ok=") + (apply_ok ? "true" : "false")) !=
                std::string::npos &&
                summary.find(std::string("disable_ok=") + (disable_ok ? "true" : "false")) !=
                    std::string::npos,
            "bench summary must expose both Apply and Disable results");

    ls2k::runtime::ShutdownResult shutdown{};
    Require(BenchExitConfirmed(result, shutdown) == (apply_ok && disable_ok),
            "main bench exit aggregation must fail for either unsuccessful stage");
}

}  // namespace

int main() {
    try {
        setenv("LS2K_BENCH_PWM_MS", "1", 1);
        setenv("LS2K_BENCH_SETTLE_MS", "0", 1);
        setenv("LS2K_BENCH_DRIVE_LEFT_PWM", "123", 1);
        setenv("LS2K_LOG_SUMMARY", "1", 1);
        CheckCase(true, true);
        CheckCase(true, false);
        CheckCase(false, true);
        CheckCase(false, false);

        BenchPwmPulseResult successful{true, true, true};
        ls2k::runtime::ShutdownResult shutdown_fail{};
        shutdown_fail.actuator_shutdown_ok = false;
        Require(!BenchExitConfirmed(successful, shutdown_fail),
                "later RunShutdown failure must also force nonzero bench exit");

        ls2k::runtime::ControlLoopTerminalResult normal_control{};
        ls2k::runtime::ShutdownResult normal_shutdown{};
        Require(RuntimeExitConfirmed(normal_control, normal_shutdown),
                "fully confirmed runtime exit must remain successful");
        normal_control.actuator_disable_ok = false;
        Require(!RuntimeExitConfirmed(normal_control, normal_shutdown),
                "normal Stop Disable failure must force nonzero main exit");
        normal_control = {};
        normal_control.timer_failure = true;
        Require(!RuntimeExitConfirmed(normal_control, normal_shutdown),
                "timer callback failure must force nonzero main exit");
        normal_control = {};
        normal_shutdown.actuator_shutdown_ok = false;
        Require(!RuntimeExitConfirmed(normal_control, normal_shutdown),
                "final Shutdown failure must force nonzero main exit");
    } catch (const std::exception& error) {
        std::cerr << "bench_pwm_pulse_result_test failed: " << error.what() << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "bench_pwm_pulse_result_test passed\n";
    return EXIT_SUCCESS;
}
