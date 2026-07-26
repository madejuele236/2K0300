#include "platform/true_ls2k0300/motor_device.hpp"

#include <cassert>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include "port/actuator_command_types.hpp"

namespace {

struct WriteEvent {
    std::string path;
    std::vector<std::uint8_t> bytes;
};

struct FakeState {
    int next_fd = 10;
    int write_count = 0;
    int fail_write = -1;
    int failures_remaining = 0;
    std::unordered_map<int, std::string> paths;
    std::vector<WriteEvent> writes;
} g;

int Open(const char* path, int, mode_t) {
    const int fd = g.next_fd++;
    g.paths[fd] = path;
    return fd;
}
ssize_t Read(int, void*, std::size_t) { return -1; }
ssize_t Write(int fd, const void* data, std::size_t size) {
    if (g.write_count++ >= g.fail_write && g.failures_remaining > 0) {
        --g.failures_remaining;
        errno = EIO;
        return -1;
    }
    const auto* begin = static_cast<const std::uint8_t*>(data);
    g.writes.push_back({g.paths.at(fd), {begin, begin + size}});
    return static_cast<ssize_t>(size);
}
off_t Seek(int, off_t, int) { return -1; }
int Close(int fd) { g.paths.erase(fd); return 0; }
int Poll(pollfd*, nfds_t, int) { return 0; }

std::uint16_t U16(const WriteEvent& event) {
    std::uint16_t value = 0;
    std::memcpy(&value, event.bytes.data(), sizeof(value));
    return value;
}

}  // namespace

int main() {
    const ls2k::platform::linux_io::SyscallApi api({&Open, &Read, &Write, &Seek, &Close, &Poll});
    const ls2k::platform::true_ls2k0300::MotorPaths paths{
        "lpwm", "rpwm", "lgpio", "rgpio", "lesc", "resc"};
    ls2k::platform::true_ls2k0300::MotorDevice motor(paths, api);

    assert(motor.Initialize().ok());
    assert(motor.Initialized());
    assert(g.writes.size() == 4);
    assert(g.writes[0].path == "lpwm" && U16(g.writes[0]) == 0);
    assert(g.writes[1].path == "rpwm" && U16(g.writes[1]) == 0);
    assert(g.writes[2].path == "lesc" && U16(g.writes[2]) == 0);
    assert(g.writes[3].path == "resc" && U16(g.writes[3]) == 0);

    g.writes.clear();
    const auto initial = motor.Apply({100, -200, -20, 1200});
    assert(initial.ok());
    assert(initial.applied_command.left_drive_pwm == 100);
    assert(initial.applied_command.right_drive_pwm == -200);
    assert(g.writes.size() == 6);
    // Logical left is cross-wired to the physical right output and inverted at
    // the hardware boundary: logical +100 selects GPIO low.
    assert(g.writes[0].path == "rgpio" && g.writes[0].bytes[0] == '0');
    assert(g.writes[1].path == "rpwm" && U16(g.writes[1]) == 100);
    // Logical right is cross-wired to the physical left output while preserving
    // the old_3 direction convention: logical -200 selects GPIO low.
    assert(g.writes[2].path == "lgpio" && g.writes[2].bytes[0] == '0');
    assert(g.writes[3].path == "lpwm" && U16(g.writes[3]) == 200);
    assert(g.writes[4].path == "lesc" && U16(g.writes[4]) == 0);
    assert(g.writes[5].path == "resc" && U16(g.writes[5]) == 1000);

    // Exact direction reversal: this cycle may only clear PWM and switch GPIO.
    g.writes.clear();
    const auto reversal = motor.Apply({-300, -200, 0, 0});
    assert(reversal.ok());
    assert(g.writes.size() == 5);
    assert(g.writes[0].path == "rpwm" && U16(g.writes[0]) == 0);
    assert(g.writes[1].path == "rgpio" && g.writes[1].bytes[0] == '1');
    assert(g.writes[2].path == "lpwm" && U16(g.writes[2]) == 200);
    assert(reversal.applied_command.left_drive_pwm == 0);
    assert(reversal.applied_command.right_drive_pwm == -200);

    // The requested reverse PWM is first eligible on the following Apply cycle.
    g.writes.clear();
    const auto reversal_next_cycle = motor.Apply({-300, -200, 0, 0});
    assert(reversal_next_cycle.ok());
    assert(g.writes.size() == 4);
    assert(g.writes[0].path == "rpwm" && U16(g.writes[0]) == 300);
    assert(g.writes[1].path == "lpwm" && U16(g.writes[1]) == 200);
    assert(reversal_next_cycle.applied_command.left_drive_pwm == -300);
    assert(reversal_next_cycle.applied_command.right_drive_pwm == -200);

    // Repeating a direction does not rewrite GPIO; drive and ESC clamps remain
    // the single authoritative conversion at the device boundary.
    g.writes.clear();
    assert(motor.Apply({-10000, -10000, 2000, -1}).ok());
    assert(g.writes.size() == 4);
    static_assert(ls2k::port::kDrivePwmDutyCapability == 9000);
    assert(g.writes[0].path == "rpwm" &&
           U16(g.writes[0]) == ls2k::port::kDrivePwmDutyCapability);
    assert(g.writes[1].path == "lpwm" &&
           U16(g.writes[1]) == ls2k::port::kDrivePwmDutyCapability);
    assert(g.writes[2].path == "lesc" && U16(g.writes[2]) == 1000);
    assert(g.writes[3].path == "resc" && U16(g.writes[3]) == 0);

    // The opposite sign transition is symmetric and also reports zero for the
    // GPIO-switch cycle before applying positive PWM on the next cycle.
    g.writes.clear();
    const auto opposite_reversal = motor.Apply({100, -10000, 0, 0});
    assert(opposite_reversal.ok());
    assert(g.writes[0].path == "rpwm" && U16(g.writes[0]) == 0);
    assert(g.writes[1].path == "rgpio" && g.writes[1].bytes[0] == '0');
    assert(opposite_reversal.applied_command.left_drive_pwm == 0);

    g.writes.clear();
    const auto opposite_next_cycle = motor.Apply({100, -10000, 0, 0});
    assert(opposite_next_cycle.ok());
    assert(g.writes[0].path == "rpwm" && U16(g.writes[0]) == 100);
    assert(opposite_next_cycle.applied_command.left_drive_pwm == 100);

    // A hot-path failure makes the object unavailable and attempts all four PWM=0 writes.
    g.writes.clear();
    g.fail_write = g.write_count;
    g.failures_remaining = 1;
    const auto failed = motor.Apply({50, 50, 50, 50});
    assert(!failed.ok());
    assert(!motor.Initialized());
    bool lpwm_zero = false, rpwm_zero = false, lesc_zero = false, resc_zero = false;
    for (const WriteEvent& write : g.writes) {
        if (write.bytes.size() != 2 || U16(write) != 0) continue;
        lpwm_zero |= write.path == "lpwm";
        rpwm_zero |= write.path == "rpwm";
        lesc_zero |= write.path == "lesc";
        resc_zero |= write.path == "resc";
    }
    assert(lpwm_zero && rpwm_zero && lesc_zero && resc_zero);
    assert(motor.Apply({}).status ==
           ls2k::platform::true_ls2k0300::MotorStatus::kNotInitialized);

    g.writes.clear();
    assert(motor.Stop().ok());
    assert(g.writes.size() == 4);
    assert(!motor.Initialized());

    // Explicit persistent mode retains the four drive PWM/GPIO descriptors.
    // A failed persistent write downgrades the whole group and retries the
    // same command through open/write/close without changing motor semantics.
    g = {};
    {
        ls2k::platform::true_ls2k0300::MotorDevice persistent_motor(
            paths,
            api,
            ls2k::platform::true_ls2k0300::MotorIoPolicy::kPreferPersistent);
        assert(persistent_motor.Initialize().ok());
        assert(persistent_motor.mode() ==
               ls2k::platform::true_ls2k0300::MotorIoMode::kPersistent);
        assert(g.paths.size() == 4);

        g.writes.clear();
        assert(persistent_motor.Apply({100, -200, 0, 0}).ok());
        assert(persistent_motor.mode() ==
               ls2k::platform::true_ls2k0300::MotorIoMode::kPersistent);

        g.writes.clear();
        g.fail_write = g.write_count;
        g.failures_remaining = 1;
        assert(persistent_motor.Apply({100, -200, 0, 0}).ok());
        assert(persistent_motor.mode() ==
               ls2k::platform::true_ls2k0300::MotorIoMode::kOpenWriteClose);
        assert(g.paths.empty());
        assert(persistent_motor.Stop().ok());
    }
}
