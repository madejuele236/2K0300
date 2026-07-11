#include "platform/camera/camera_service_internal.hpp"
#include "inference/classifier_service_internal.hpp"
#include "transport/stream_service_internal.hpp"

// Preserve the baseline image.cpp construction order in one composition TU.
LQ_NCNN classifier;
TransmissionStreamServer camera_server;
lq_camera_ex cam(320,240,156,LQ_CAMERA_0CPU_MJPG,LQ_CAMERA_PATH);

namespace primer::inference {
LQ_NCNN &Classifier() { return classifier; }
}  // namespace primer::inference

namespace primer::transport {
TransmissionStreamServer &CameraStreamServer() { return camera_server; }
}  // namespace primer::transport

namespace primer::platform {
lq_camera_ex &Camera() { return cam; }
void StopCamera() { cam.stop_collect(); }
}  // namespace primer::platform
