#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "platform/bootstrap.hpp"

namespace {

class Diagnostics final : public ls2k::port::DiagnosticSink {
public:
    void Emit(const ls2k::port::DiagnosticEvent& event) override {
        saw_validation = saw_validation || event.code == "params.validation";
    }

    bool saw_validation = false;
};

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

std::string Fixture(const std::string& optional_fields,
                    const std::string& assistant_port = "8888") {
    return
        "{\n"
        "  \"RUNNING_SPEED_TARGET\": 300,\n"
        "  \"YAW_RATE_PID\": {\"P\": 12, \"I\": 0, \"D\": 0},\n"
        "  \"LEFT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1000},\n"
        "  \"RIGHT_WHEEL_PID\": {\"P\": 0, \"I\": 0, \"D\": 0, \"INTEGRAL_LIMIT\": 1000},\n"
        "  \"assistant_tcp\": {\"host\": \"127.0.0.1\", \"port\": " + assistant_port + "}" +
        (optional_fields.empty() ? "\n" : ",\n  " + optional_fields + "\n") +
        "}\n";
}

void Write(const std::string& path, const std::string& text) {
    std::ofstream output(path);
    Expect(output.is_open(), "cannot write probe fixture: " + path);
    output << text;
}

void ExpectRejected(const std::string& name,
                    const std::string& optional_fields,
                    const std::string& assistant_port = "8888") {
    const std::string path = "/tmp/param_store_semantic_boundary_" + name + ".json";
    Write(path, Fixture(optional_fields, assistant_port));
    ls2k::port::RuntimeParameters output{};
    output.running_speed_target = -101.0;
    output.control_period_ms = -202;
    output.assistant_tcp.port = -303;
    output.steering_media_port = -404;
    Diagnostics diagnostics{};
    const auto store = ls2k::platform::MakeParamStore();
    Expect(store != nullptr, "MakeParamStore returned null");
    Expect(!store->LoadRuntimeParameters(path, output, diagnostics),
           name + " unexpectedly passed the production loader");
    Expect(diagnostics.saw_validation, name + " did not emit params.validation");
    Expect(output.running_speed_target == -101.0 &&
               output.control_period_ms == -202 &&
               output.assistant_tcp.port == -303 &&
               output.steering_media_port == -404,
           name + " changed output on rejection");
}

void ExpectAccepted(const std::string& name,
                    const std::string& optional_fields,
                    const std::string& assistant_port = "8888") {
    const std::string path = "/tmp/param_store_semantic_boundary_" + name + ".json";
    Write(path, Fixture(optional_fields, assistant_port));
    ls2k::port::RuntimeParameters output{};
    Diagnostics diagnostics{};
    const auto store = ls2k::platform::MakeParamStore();
    Expect(store != nullptr, "MakeParamStore returned null");
    Expect(store->LoadRuntimeParameters(path, output, diagnostics),
           name + " unexpectedly failed the production loader");
}

}  // namespace

int main() {
    try {
        ExpectRejected("control_period_negative", "\"control_period_ms\": -7");
        ExpectRejected("control_period_int_max", "\"control_period_ms\": 2147483647");
        ExpectRejected("assistant_port_negative", "", "-1");
        ExpectRejected("assistant_port_65536", "", "65536");
        ExpectRejected("media_port_65536", "\"steering_media_enabled\": 0, \"steering_media_port\": 65536");
        ExpectRejected("motion_turn_nonfinite", "\"motion_turn_limit_spinup\": 1e999");

        ExpectAccepted("control_period_min", "\"control_period_ms\": 1");
        ExpectAccepted("control_period_max", "\"control_period_ms\": 1000");
        ExpectAccepted("assistant_port_min", "", "1");
        ExpectAccepted("assistant_port_max", "", "65535");
        ExpectAccepted("media_port_max", "\"steering_media_enabled\": 0, \"steering_media_port\": 65535");
        ExpectAccepted("motion_turn_min", "\"motion_turn_limit_spinup\": 0");
        ExpectAccepted("motion_turn_max", "\"motion_turn_limit_spinup\": 1");

        std::cout << "param_store_semantic_boundary_probe passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "param_store_semantic_boundary_probe failed: " << error.what() << '\n';
        return 1;
    }
}
