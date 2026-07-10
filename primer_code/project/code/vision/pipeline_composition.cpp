#include "platform/camera/camera_service.hpp"
#include "inference/classifier_service.hpp"
#include "transport/stream_service.hpp"

LQ_NCNN classifier;
TransmissionStreamServer camera_server;
lq_camera_ex cam(320,240,156,LQ_CAMERA_0CPU_MJPG,LQ_CAMERA_PATH);
