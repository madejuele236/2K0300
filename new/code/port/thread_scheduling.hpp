#ifndef LS2K_PORT_THREAD_SCHEDULING_HPP
#define LS2K_PORT_THREAD_SCHEDULING_HPP

#include "port/diagnostics.hpp"

namespace ls2k::port {

enum class ThreadSchedulingRole {
    kMainLoop,
    kControlTimer,
    kCameraCapture,
    kSteeringMedia,
};

bool ApplyThreadSchedulingProfile(ThreadSchedulingRole role,
                                  DiagnosticSink* diagnostics = nullptr);

}  // namespace ls2k::port

#endif  // LS2K_PORT_THREAD_SCHEDULING_HPP
