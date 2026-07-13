#include "platform/true_ls2k0300/bridge.hpp"

#include <chrono>
#include <cstring>
#include <iostream>
#include <thread>

namespace {

using ls2k::platform::true_ls2k0300::BridgeStatus;

void PrintStatus(const char* step, const BridgeStatus& status) {
    std::cout << step << ": ok=" << (status.ok ? "true" : "false")
              << ", detail=" << status.detail << std::endl;
}

}  // namespace

int main(int argc, char** argv) {
    constexpr const char* kConfirmation = "--confirm-baseline-dual-5s";
    if (argc != 2 || std::strcmp(argv[1], kConfirmation) != 0) {
        std::cerr << "refusing to access motor outputs: pass exactly " << kConfirmation
                  << std::endl;
        return 2;
    }

    std::cout << "step 1/4: InitializeMotor()" << std::endl;
    const BridgeStatus initialize =
        ls2k::platform::true_ls2k0300::InitializeMotor();
    PrintStatus("InitializeMotor()", initialize);

    if (!initialize.ok) {
        std::cout << "step 4/4: DisableMotorOutput() after initialization failure"
                  << std::endl;
        const BridgeStatus disable =
            ls2k::platform::true_ls2k0300::DisableMotorOutput();
        PrintStatus("DisableMotorOutput()", disable);
        return disable.ok ? 3 : 4;
    }

    std::cout << "step 2/4: ApplyMotorCommand(3000, 3000)" << std::endl;
    const BridgeStatus apply =
        ls2k::platform::true_ls2k0300::ApplyMotorCommand(3000, 3000);
    const auto hold_deadline = std::chrono::steady_clock::now() +
                               std::chrono::seconds(5);
    PrintStatus("ApplyMotorCommand(3000, 3000)", apply);

    int result = 0;
    if (apply.ok) {
        std::cout << "step 3/4: holding both commands for 5 seconds" << std::endl;
        std::this_thread::sleep_until(hold_deadline);
    } else {
        std::cout << "step 3/4: hold skipped because command application failed"
                  << std::endl;
        result = 5;
    }

    std::cout << "step 4/4: DisableMotorOutput()" << std::endl;
    const BridgeStatus disable =
        ls2k::platform::true_ls2k0300::DisableMotorOutput();
    PrintStatus("DisableMotorOutput()", disable);
    if (!disable.ok) {
        result = 6;
    }
    return result;
}
