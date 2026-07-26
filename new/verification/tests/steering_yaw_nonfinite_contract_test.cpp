#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <vector>

#include "control/steering_yaw_controller.hpp"
#include "control/wheel_target_mixer.hpp"
#include "platform/bootstrap.hpp"

namespace {

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

class CaptureDiagnostics final : public ls2k::port::DiagnosticSink {
public:
    void Emit(const ls2k::port::DiagnosticEvent& event) override { events.push_back(event); }
    std::vector<ls2k::port::DiagnosticEvent> events{};
};

ls2k::port::RuntimeParameters LoadProductionParameters(const std::string& path) {
    CaptureDiagnostics diagnostics{};
    ls2k::port::RuntimeParameters params{};
    const std::unique_ptr<ls2k::port::IParamStore> store = ls2k::platform::MakeParamStore();
    Require(store != nullptr, "MakeParamStore must return the production loader");
    Require(store->LoadRuntimeParameters(path, params, diagnostics),
            "production loader must accept the active parameter file");
    return params;
}

void TestLoadedControllerAcrossProductionInputSigns(const std::string& path) {
    const ls2k::port::RuntimeParameters params = LoadProductionParameters(path);
    ls2k::control::SteeringYawController controller{};
    Require(controller.Configure(params), "loader-accepted yaw parameters must configure production controller");

    for (float target : {-9000.0F, 0.0F, 9000.0F}) {
        for (float gyro : {-ls2k::port::kMaximumProducedGyroMagnitudeRadPerSec,
                           0.0F,
                           ls2k::port::kMaximumProducedGyroMagnitudeRadPerSec}) {
            ls2k::port::BEVControllerMemory memory{};
            const auto result = controller.ComputeGyroTurn(target, gyro, memory);
            Require(result.valid && result.status == ls2k::control::SteeringYawStatus::kOk,
                    "loaded controller must accept signed production target/gyro inputs");
            Require(std::isfinite(result.raw_turn_output) &&
                        std::abs(result.raw_turn_output) <=
                            ls2k::control::kAppliedTurnOutputMagnitudeLimit,
                    "valid yaw output must remain finite and within the shared 9000 bound");
        }
    }
}

float MaximumAcceptedGain(char component, float denominator) {
    float gain = std::numeric_limits<float>::max() / denominator;
    const auto accepted = [component](float candidate) {
        return ls2k::control::YawRatePidArithmeticIsFinite(
            component == 'P' ? candidate : 0.0,
            component == 'I' ? candidate : 0.0,
            component == 'D' ? candidate : 0.0);
    };
    while (!accepted(gain)) {
        gain = std::nextafter(gain, 0.0F);
    }
    while (accepted(std::nextafter(gain, std::numeric_limits<float>::infinity()))) {
        gain = std::nextafter(gain, std::numeric_limits<float>::infinity());
    }
    return gain;
}

std::string WriteBoundaryFixture(const std::string& source_path,
                                 char component,
                                 float gain) {
    std::ifstream input(source_path);
    Require(input.is_open(), "active parameter file must be readable");
    std::ostringstream buffer;
    buffer << input.rdbuf();
    std::string text = buffer.str();
    const std::string yaw_block =
        "\"YAW_RATE_PID\": {\n    \"P\": 0.0,\n    \"I\": 0.0,\n    \"D\": 0.0\n  }";
    std::ostringstream replacement;
    replacement << std::setprecision(std::numeric_limits<float>::max_digits10)
                << "\"YAW_RATE_PID\": {\n    \"P\": "
                << (component == 'P' ? gain : 0.0F)
                << ",\n    \"I\": " << (component == 'I' ? gain : 0.0F)
                << ",\n    \"D\": " << (component == 'D' ? gain : 0.0F) << "\n  }";
    const std::size_t position = text.find(yaw_block);
    Require(position != std::string::npos, "active YAW_RATE_PID block shape changed");
    text.replace(position, yaw_block.size(), replacement.str());
    const std::string output_path =
        "/tmp/ls2k_steering_yaw_boundary_" + std::string(1, component) + ".json";
    std::ofstream output(output_path);
    Require(output.is_open(), "boundary fixture must be writable");
    output << text;
    return output_path;
}

void TestPidBoundaryArithmetic(const std::string& active_params_path) {
    const float error = ls2k::control::kMaximumYawErrorMagnitude;
    const float derivative = ls2k::control::kMaximumYawDerivativeInputMagnitude;
    const float integral = ls2k::control::kYawIntegralAccumulatorMagnitudeLimit;
    struct Case { char component; float denominator; };
    for (const Case test_case : {Case{'P', error}, Case{'I', integral}, Case{'D', derivative}}) {
        float gain = MaximumAcceptedGain(test_case.component, test_case.denominator);
        const auto accepted = [test_case](float candidate) {
            return ls2k::control::YawRatePidArithmeticIsFinite(
                test_case.component == 'P' ? candidate : 0.0,
                test_case.component == 'I' ? candidate : 0.0,
                test_case.component == 'D' ? candidate : 0.0);
        };
        Require(accepted(gain), "exact adjacent accepted PID boundary must pass");
        Require(!accepted(std::nextafter(gain, std::numeric_limits<float>::infinity())),
                "next representable PID gain must fail the production arithmetic contract");

        const ls2k::port::RuntimeParameters loaded =
            LoadProductionParameters(WriteBoundaryFixture(active_params_path,
                                                          test_case.component,
                                                          gain));
        ls2k::control::SteeringYawController controller{};
        Require(controller.Configure(loaded),
                "MakeParamStore accepted boundary must configure SteeringYawController");
        ls2k::port::BEVControllerMemory memory{};
        float gyro = 0.0F;
        if (test_case.component == 'I') {
            memory.gyro_i_accumulator = ls2k::control::kYawIntegralAccumulatorMagnitudeLimit;
        } else if (test_case.component == 'D') {
            memory.gyro_error_last = -ls2k::control::kMaximumYawErrorMagnitude;
            gyro = -ls2k::control::kMaximumYawErrorMagnitude;
        } else {
            gyro = -ls2k::control::kMaximumYawErrorMagnitude;
        }
        const auto result = controller.ComputeGyroTurn(
            ls2k::control::kMaximumTurnOutputTargetMagnitude, gyro, memory);
        Require(result.valid && std::isfinite(result.raw_turn_output),
                "accepted loader/controller boundary must produce a finite bounded result");
    }
}

void TestProgrammaticViolationsFailClosed() {
    ls2k::port::RuntimeParameters params{};
    params.yaw_rate_pid_d = 1.0e308;
    ls2k::control::SteeringYawController controller{};
    Require(!controller.Configure(params), "programmatic 1e308 yaw gain must fail configuration");
    ls2k::port::BEVControllerMemory memory{};
    memory.gyro_error_last = 3.0F;
    const auto invalid_config = controller.ComputeGyroTurn(1.0F, 0.0F, memory);
    Require(!invalid_config.valid &&
                invalid_config.status == ls2k::control::SteeringYawStatus::kInvalidConfiguration,
            "invalid configuration must remain observable at compute time");
    Require(memory.gyro_error_last == 3.0F,
            "invalid configuration must not mutate controller memory");

    params = {};
    Require(controller.Configure(params), "default programmatic configuration must be valid");
    const auto invalid_gyro = controller.ComputeGyroTurn(
        1.0F, std::numeric_limits<float>::quiet_NaN(), memory);
    Require(!invalid_gyro.valid &&
                invalid_gyro.status == ls2k::control::SteeringYawStatus::kInvalidGyroInput,
            "nonfinite runtime gyro must fail before arithmetic");
    const auto out_of_contract_gyro = controller.ComputeGyroTurn(
        1.0F,
        std::nextafter(ls2k::port::kMaximumProducedGyroMagnitudeRadPerSec,
                       std::numeric_limits<float>::infinity()),
        memory);
    Require(!out_of_contract_gyro.valid &&
                out_of_contract_gyro.status == ls2k::control::SteeringYawStatus::kInvalidGyroInput,
            "finite gyro outside the producer range must fail observably");

    memory.gyro_i_accumulator = std::numeric_limits<float>::infinity();
    const auto invalid_memory = controller.ComputeGyroTurn(1.0F, 0.0F, memory);
    Require(!invalid_memory.valid &&
                invalid_memory.status == ls2k::control::SteeringYawStatus::kInvalidControllerMemory,
            "nonfinite controller memory must fail before arithmetic");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        Require(argc == 2, "usage: steering_yaw_nonfinite_contract_test <default_params.json>");
        TestLoadedControllerAcrossProductionInputSigns(argv[1]);
        TestPidBoundaryArithmetic(argv[1]);
        TestProgrammaticViolationsFailClosed();
        std::cout << "steering_yaw_nonfinite_contract_test passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "steering_yaw_nonfinite_contract_test failed: " << error.what() << '\n';
        return 1;
    }
}
