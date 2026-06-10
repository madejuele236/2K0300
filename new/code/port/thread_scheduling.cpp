#include "port/thread_scheduling.hpp"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <pthread.h>
#include <sched.h>
#include <string>

namespace ls2k::port {
namespace {

constexpr int kControlTimerPriorityOffset = 8;
constexpr int kMainLoopPriorityOffset = 10;
constexpr int kCameraCapturePriorityOffset = 12;
constexpr int kSteeringMediaPriorityOffset = 14;

bool EnvDisabled(const char* key) {
    const char* value = std::getenv(key);
    if (value == nullptr || value[0] == '\0') {
        return false;
    }
    return value[0] == '0' || value[0] == 'n' || value[0] == 'N' ||
           value[0] == 'f' || value[0] == 'F';
}

const char* RoleName(ThreadSchedulingRole role) {
    switch (role) {
        case ThreadSchedulingRole::kMainLoop:
            return "main_loop";
        case ThreadSchedulingRole::kControlTimer:
            return "control_timer";
        case ThreadSchedulingRole::kCameraCapture:
            return "camera_capture";
        case ThreadSchedulingRole::kSteeringMedia:
            return "steering_media";
    }
    return "unknown";
}

const char* ThreadName(ThreadSchedulingRole role) {
    switch (role) {
        case ThreadSchedulingRole::kMainLoop:
            return "ls2k-main";
        case ThreadSchedulingRole::kControlTimer:
            return "ls2k-control";
        case ThreadSchedulingRole::kCameraCapture:
            return "ls2k-camera";
        case ThreadSchedulingRole::kSteeringMedia:
            return "ls2k-media";
    }
    return "ls2k";
}

int PriorityOffset(ThreadSchedulingRole role) {
    switch (role) {
        case ThreadSchedulingRole::kControlTimer:
            return kControlTimerPriorityOffset;
        case ThreadSchedulingRole::kMainLoop:
            return kMainLoopPriorityOffset;
        case ThreadSchedulingRole::kCameraCapture:
            return kCameraCapturePriorityOffset;
        case ThreadSchedulingRole::kSteeringMedia:
            return kSteeringMediaPriorityOffset;
    }
    return kSteeringMediaPriorityOffset;
}

void EmitSchedulingDiagnostic(DiagnosticSink* diagnostics,
                              DiagnosticLevel level,
                              const std::string& message) {
    if (diagnostics == nullptr) {
        return;
    }
    diagnostics->Emit({level, "thread.scheduling", message, NowMs()});
}

}  // namespace

bool ApplyThreadSchedulingProfile(ThreadSchedulingRole role,
                                  DiagnosticSink* diagnostics) {
    pthread_setname_np(pthread_self(), ThreadName(role));
    if (EnvDisabled("LS2K_THREAD_SCHEDULING")) {
        EmitSchedulingDiagnostic(diagnostics,
                                 DiagnosticLevel::kInfo,
                                 std::string("thread scheduling disabled for role=") + RoleName(role));
        return false;
    }

    const int max_priority = sched_get_priority_max(SCHED_FIFO);
    const int min_priority = sched_get_priority_min(SCHED_FIFO);
    if (max_priority < 0 || min_priority < 0) {
        EmitSchedulingDiagnostic(diagnostics,
                                 DiagnosticLevel::kWarning,
                                 std::string("failed to query SCHED_FIFO priority range for role=") +
                                     RoleName(role) + " errno=" + std::to_string(errno));
        return false;
    }

    sched_param param{};
    param.sched_priority =
        std::max(min_priority, max_priority - PriorityOffset(role));
    const int set_rc = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (set_rc != 0) {
        EmitSchedulingDiagnostic(diagnostics,
                                 DiagnosticLevel::kWarning,
                                 std::string("failed to apply SCHED_FIFO for role=") +
                                     RoleName(role) + " priority=" +
                                     std::to_string(param.sched_priority) +
                                     " errno=" + std::to_string(set_rc) +
                                     " error=" + std::strerror(set_rc));
        return false;
    }

    EmitSchedulingDiagnostic(diagnostics,
                             DiagnosticLevel::kInfo,
                             std::string("applied SCHED_FIFO for role=") +
                                 RoleName(role) + " priority=" +
                                 std::to_string(param.sched_priority));
    return true;
}

}  // namespace ls2k::port
