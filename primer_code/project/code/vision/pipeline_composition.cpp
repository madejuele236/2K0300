#include "internal/camera_service.hpp"
#include "internal/classifier_service.hpp"
#include "internal/transport_service.hpp"

LQ_NCNN classifier;
TransmissionStreamServer camera_server;
lq_camera_ex cam(320,240,156,LQ_CAMERA_0CPU_MJPG,LQ_CAMERA_PATH);
