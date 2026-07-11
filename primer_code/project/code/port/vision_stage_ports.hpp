#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace primer::port {

class VisionClassifierPort {
public:
    virtual ~VisionClassifierPort() = default;
    virtual void SetModelPath(const std::string &param_path,
                              const std::string &bin_path) = 0;
    virtual void SetInputSize(int width, int height) = 0;
    virtual void SetLabels(const std::vector<std::string> &labels) = 0;
    virtual void SetNormalize(const float mean_vals[3],
                              const float norm_vals[3]) = 0;
    virtual bool Init() = 0;
    virtual std::string Infer(const cv::Mat &image, float &confidence) = 0;
};

class VisionCameraPort {
public:
    virtual ~VisionCameraPort() = default;
    virtual void start_collect() = 0;
    virtual cv::Mat get_frame_raw() = 0;
    virtual bool is_cam_opened() const = 0;
    virtual void set_exposure_manual(int exposure) = 0;
    virtual int get_camera_width() const = 0;
    virtual int get_camera_height() const = 0;
    virtual int get_camera_fps() const = 0;
};

class VisionStreamPort {
public:
    virtual ~VisionStreamPort() = default;
    virtual void start_server(int port) = 0;
};

struct VisionParametersPort {
    const int &debug_rgb_r_min;
    const int &debug_rgb_rb_diff;
    const int &debug_rgb_rg_diff;
};

class VisionDirectionControllerPort {
public:
    explicit VisionDirectionControllerPort(float &proportional_gain)
        : Kp(proportional_gain) {}
    virtual ~VisionDirectionControllerPort() = default;
    virtual float Calculate(float expect, float feedback) = 0;

    float &Kp;
};

struct VisionEncoderPort {
    const int16_t &count_now;
};

VisionClassifierPort &VisionClassifier();
VisionCameraPort &VisionCamera();
VisionStreamPort &VisionStream();
const VisionParametersPort &VisionParameters();
VisionDirectionControllerPort &VisionDirectionController();
float CalculateVisionDirection(VisionDirectionControllerPort *controller,
                               float expect, float feedback);
float &MutableVisionImageOutput();
float VisionMasterSpeed();
float VisionCurrentSpeed();
const VisionEncoderPort &VisionLeftEncoder();
const VisionEncoderPort &VisionRightEncoder();
void SetVisionEscDuty(int duty);
void SetVisionRunMode(int8_t value);
int16_t VisionDistanceRaw();

}  // namespace primer::port
