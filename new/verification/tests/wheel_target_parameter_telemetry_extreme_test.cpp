#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>

#include "control/wheel_target_mixer.hpp"
#include "observability/assistant_telemetry_view.hpp"
#include "observability/control_debug_snapshot.hpp"
#include "platform/bootstrap.hpp"
#include "port/diagnostics.hpp"
#include "port/runtime_parameter_types.hpp"
#include "transport/assistant_protocol.hpp"

namespace {

class CaptureDiagnostics final : public ls2k::port::DiagnosticSink {
public:
    void Emit(const ls2k::port::DiagnosticEvent& event) override {
        last_code = event.code;
        last_message = event.message;
    }

    std::string last_code{};
    std::string last_message{};
};

void Require(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::string ReadText(const std::string& path) {
    std::ifstream input(path);
    Require(input.is_open(), "failed to open template config: " + path);
    std::ostringstream text;
    text << input.rdbuf();
    return text.str();
}

void WriteText(const std::string& path, const std::string& text) {
    std::ofstream output(path);
    Require(output.is_open(), "failed to write config fixture: " + path);
    output << text;
}

std::string JsonNumber(double value) {
    std::ostringstream text;
    text << std::setprecision(std::numeric_limits<double>::max_digits10) << value;
    return text.str();
}

std::string ReplaceJsonNumber(std::string json,
                              const std::string& key,
                              const std::string& replacement,
                              std::size_t search_from = 0U) {
    const std::string marker = "\"" + key + "\"";
    const std::size_t key_position = json.find(marker, search_from);
    Require(key_position != std::string::npos, "fixture key not found: " + key);
    const std::size_t colon = json.find(':', key_position + marker.size());
    Require(colon != std::string::npos, "fixture colon not found: " + key);
    const std::size_t value_begin = json.find_first_not_of(" \t\r\n", colon + 1U);
    const std::size_t value_end = json.find_first_of(",}\r\n", value_begin);
    Require(value_begin != std::string::npos && value_end != std::string::npos,
            "fixture numeric value not found: " + key);
    json.replace(value_begin, value_end - value_begin, replacement);
    return json;
}

std::string BuildConfig(const std::string& template_json,
                        double running_speed,
                        int raw_turn_limit,
                        double accel_scale,
                        double decel_scale,
                        double ml_speed) {
    std::string json = ReplaceJsonNumber(
        template_json, "RUNNING_SPEED_TARGET", JsonNumber(running_speed));
    json = ReplaceJsonNumber(json, "raw_turn_output_limit", std::to_string(raw_turn_limit));
    json = ReplaceJsonNumber(
        json, "wheel_turn_accel_delta_scale", JsonNumber(accel_scale));
    json = ReplaceJsonNumber(
        json, "wheel_turn_decel_delta_scale", JsonNumber(decel_scale));
    return ReplaceJsonNumber(json, "SPEED_TARGET", JsonNumber(ml_speed));
}

std::string BuildEnabledMlConfig(const std::string& template_json,
                                 double ml_speed) {
    std::string json = BuildConfig(template_json, 5000.0, 9000, 1.0, 1.0, ml_speed);
    const std::size_t ml_position = json.find("\"ML\"");
    Require(ml_position != std::string::npos, "ML block not found");
    json = ReplaceJsonNumber(json, "ENABLED", "1", ml_position);
    json = ReplaceJsonNumber(json, "ENCODER_TICKS_TO_METER", "0.001");
    json = ReplaceJsonNumber(json, "EXIT_FORWARD_M", "0.5", ml_position);
    json = ReplaceJsonNumber(json, "MAX_DURATION_MS", "1000", ml_position);
    json = ReplaceJsonNumber(json, "MAX_INTEGRATION_GAP_MS", "50", ml_position);
    return json;
}

ls2k::port::RuntimeParameters LoadAccepted(const std::string& fixture_path,
                                           const std::string& json) {
    WriteText(fixture_path, json);
    CaptureDiagnostics diagnostics{};
    ls2k::port::RuntimeParameters params{};
    const std::unique_ptr<ls2k::port::IParamStore> store = ls2k::platform::MakeParamStore();
    Require(store != nullptr, "MakeParamStore returned null");
    Require(store->LoadRuntimeParameters(fixture_path, params, diagnostics),
            "expected accepted config, diagnostic=" + diagnostics.last_code + " " +
                diagnostics.last_message);
    Require(diagnostics.last_code == "params.loaded",
            "accepted config must emit params.loaded");
    return params;
}

void ExpectRejectedUnchanged(const std::string& fixture_path,
                             const std::string& json,
                             const std::string& label) {
    WriteText(fixture_path, json);
    CaptureDiagnostics diagnostics{};
    ls2k::port::RuntimeParameters params{};
    params.running_speed_target = -123.0;
    params.raw_turn_output_limit = -456;
    params.wheel_turn_accel_delta_scale = -789.0;
    params.wheel_turn_decel_delta_scale = -987.0;
    params.ml.maneuver.speed_target = -654.0;
    params.assistant_tcp.host = "unchanged-sentinel";
    const std::unique_ptr<ls2k::port::IParamStore> store = ls2k::platform::MakeParamStore();
    Require(store != nullptr, "MakeParamStore returned null");
    Require(!store->LoadRuntimeParameters(fixture_path, params, diagnostics),
            label + " must be rejected");
    Require(diagnostics.last_code == "params.validation",
            label + " must emit params.validation");
    Require(params.running_speed_target == -123.0 &&
                params.raw_turn_output_limit == -456 &&
                params.wheel_turn_accel_delta_scale == -789.0 &&
                params.wheel_turn_decel_delta_scale == -987.0 &&
                params.ml.maneuver.speed_target == -654.0 &&
                params.assistant_tcp.host == "unchanged-sentinel",
            label + " must leave the complete representative output sentinel unchanged");
}

double LargestSafeScale(double base_target, int raw_turn_limit) {
    const double turn =
        ls2k::control::MaximumAppliedTurnOutputMagnitude(raw_turn_limit);
    Require(turn > 0.0, "boundary search requires reachable nonzero turn");
    double scale = (std::numeric_limits<double>::max() - base_target) / turn;
    const ls2k::control::WheelTargetMixerParameters params{scale, 0.0};
    if (!ls2k::control::WheelTargetArithmeticIsFinite(base_target, raw_turn_limit, params)) {
        scale = std::nextafter(scale, 0.0);
    }
    while (!ls2k::control::WheelTargetArithmeticIsFinite(
        base_target, raw_turn_limit, {scale, 0.0})) {
        scale = std::nextafter(scale, 0.0);
    }
    return scale;
}

std::string EncodeTargets(const ls2k::control::WheelSpeedTargets& targets,
                          double effective_speed_target,
                          int applied_turn_output) {
    Require(std::isfinite(targets.left) && std::isfinite(targets.right),
            "production mixer targets must be finite before telemetry mapping");
    ls2k::observability::ControlDebugSnapshot snapshot{};
    snapshot.valid = true;
    snapshot.steering.effective_speed_target = effective_speed_target;
    snapshot.left_speed_target = targets.left;
    snapshot.right_speed_target = targets.right;
    snapshot.raw_turn_output = applied_turn_output;
    snapshot.applied_turn_output = applied_turn_output;
    return ls2k::transport::EncodeAssistantTelemetry(
        ls2k::observability::BuildAssistantTelemetryView(snapshot));
}

void TestBoundaryAndTelemetry(const std::string& template_json,
                              const std::string& fixture_path) {
    constexpr int kRawTurnLimit = 9000;
    constexpr double kBaseTarget = ls2k::control::kWheelSpeedTargetMax;
    const double accel_scale = LargestSafeScale(kBaseTarget, kRawTurnLimit);
    const double decel_scale = accel_scale;
    const ls2k::port::RuntimeParameters params = LoadAccepted(
        fixture_path,
        BuildConfig(template_json,
                    5000.0,
                    kRawTurnLimit,
                    accel_scale,
                    decel_scale,
                    0.0));
    Require(ls2k::control::WheelTargetArithmeticIsFinite(
                kBaseTarget,
                params.raw_turn_output_limit,
                {params.wheel_turn_accel_delta_scale,
                 params.wheel_turn_decel_delta_scale}),
            "loaded near-boundary config must satisfy production arithmetic contract");

    ls2k::control::WheelTargetMixer mixer{};
    const auto positive = mixer.Compute(
        kBaseTarget,
        kRawTurnLimit,
        {params.wheel_turn_accel_delta_scale, params.wheel_turn_decel_delta_scale});
    const auto negative = mixer.Compute(
        kBaseTarget,
        -kRawTurnLimit,
        {params.wheel_turn_accel_delta_scale, params.wheel_turn_decel_delta_scale});
    Require(positive.left == negative.right && positive.right == negative.left,
            "positive/negative turn must cover mirrored accel/decel wheel paths");
    std::cout << EncodeTargets(positive, kBaseTarget, kRawTurnLimit) << '\n';
    std::cout << EncodeTargets(negative, kBaseTarget, -kRawTurnLimit) << '\n';

    const double unsafe_accel = std::nextafter(accel_scale,
                                                std::numeric_limits<double>::infinity());
    const double unsafe_accel_delta =
        ls2k::control::MaximumAppliedTurnOutputMagnitude(kRawTurnLimit) * unsafe_accel;
    Require(!std::isfinite(unsafe_accel_delta),
            "next accel scale above production boundary must overflow multiplication");
    Require(!ls2k::control::WheelTargetArithmeticIsFinite(
                kBaseTarget, kRawTurnLimit, {unsafe_accel, 0.0}),
            "next accel scale above accepted boundary must be rejected");
    ExpectRejectedUnchanged(
        fixture_path,
        BuildConfig(template_json, 5000.0, kRawTurnLimit, unsafe_accel, 0.0, 0.0),
        "accel multiplication overflow");

    const double unsafe_decel = std::nextafter(decel_scale,
                                                std::numeric_limits<double>::infinity());
    Require(!std::isfinite(
                ls2k::control::MaximumAppliedTurnOutputMagnitude(kRawTurnLimit) * unsafe_decel),
            "next decel scale above accepted boundary must overflow multiplication");
    ExpectRejectedUnchanged(
        fixture_path,
        BuildConfig(template_json, 5000.0, kRawTurnLimit, 0.0, unsafe_decel, 0.0),
        "decel multiplication overflow");
}

void TestAdditionOverflowGuard() {
    constexpr int kRawTurnLimit = 9000;
    const double base_target = std::numeric_limits<double>::max() / 2.0;
    const double accel_scale = LargestSafeScale(base_target, kRawTurnLimit);
    const double unsafe_accel = std::nextafter(accel_scale,
                                                std::numeric_limits<double>::infinity());
    const double delta =
        ls2k::control::MaximumAppliedTurnOutputMagnitude(kRawTurnLimit) * unsafe_accel;
    Require(std::isfinite(delta) && !std::isfinite(base_target + delta),
            "shared contract must detect addition overflow after finite multiplication");
    Require(!ls2k::control::WheelTargetArithmeticIsFinite(
                base_target, kRawTurnLimit, {unsafe_accel, 0.0}),
            "addition overflow must fail the shared arithmetic contract");
}

void TestReportedCounterexampleAndZeroTurnDomain(const std::string& template_json,
                                                 const std::string& fixture_path) {
    ExpectRejectedUnchanged(
        fixture_path,
        BuildConfig(template_json, 5000.0, 20000, 1.0e308, 1.0, 0.0),
        "reported 1e308 accel scale counterexample");
    ExpectRejectedUnchanged(
        fixture_path,
        BuildConfig(template_json, 5000.0, -1, 1.0, 1.0, 0.0),
        "negative raw turn limit");

    const auto disabled_ml_ignored = LoadAccepted(
        fixture_path,
        BuildConfig(template_json, 5000.0, 9000, 1.0, 1.0, -1.0));
    Require(!disabled_ml_ignored.ml.enabled,
            "disabled ML speed must remain unreachable and not expand rejection domain");
    const auto enabled_ml_boundary =
        LoadAccepted(fixture_path, BuildEnabledMlConfig(template_json, 5000.0));
    Require(enabled_ml_boundary.ml.enabled &&
                enabled_ml_boundary.ml.maneuver.speed_target == 5000.0,
            "enabled ML speed must accept the shared 5000 boundary");
    ExpectRejectedUnchanged(
        fixture_path,
        BuildEnabledMlConfig(template_json, std::nextafter(5000.0,
                                                            std::numeric_limits<double>::infinity())),
        "enabled ML speed above shared wheel base boundary");

    const ls2k::port::RuntimeParameters zero_turn = LoadAccepted(
        fixture_path,
        BuildConfig(template_json,
                    5000.0,
                    0,
                    std::numeric_limits<double>::max(),
                    std::numeric_limits<double>::max(),
                    0.0));
    ls2k::control::WheelTargetMixer mixer{};
    const auto targets = mixer.Compute(
        zero_turn.running_speed_target,
        0,
        {zero_turn.wheel_turn_accel_delta_scale,
         zero_turn.wheel_turn_decel_delta_scale});
    Require(targets.left == 5000.0 && targets.right == 5000.0,
            "raw turn limit zero makes the full finite scale domain safe");
    std::cout << EncodeTargets(targets, zero_turn.running_speed_target, 0) << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: wheel_target_parameter_telemetry_extreme_test "
                     "<default_params.json> <fixture.json>\n";
        return EXIT_FAILURE;
    }
    try {
        const std::string template_json = ReadText(argv[1]);
        TestAdditionOverflowGuard();
        TestBoundaryAndTelemetry(template_json, argv[2]);
        TestReportedCounterexampleAndZeroTurnDomain(template_json, argv[2]);
        std::remove(argv[2]);
    } catch (const std::exception& error) {
        std::cerr << "wheel_target_parameter_telemetry_extreme_test failed: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
