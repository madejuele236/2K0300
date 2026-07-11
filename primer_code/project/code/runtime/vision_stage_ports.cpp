#include "port/vision_stage_ports.hpp"

#include "control/pid_controller.h"
#include "inference/classifier_service.hpp"
#include "parameters/parameter_store.h"
#include "platform/camera/camera_service.hpp"
#include "platform/device_platform.h"
#include "port/run_state_port.hpp"
#include "transport/stream_service.hpp"

namespace primer::port {
namespace {

class ClassifierAdapter final : public VisionClassifierPort {
public:
    void SetModelPath(const std::string &param_path,
                      const std::string &bin_path) override
    {
        inference::Classifier().SetModelPath(param_path, bin_path);
    }
    void SetInputSize(int width, int height) override
    {
        inference::Classifier().SetInputSize(width, height);
    }
    void SetLabels(const std::vector<std::string> &labels) override
    {
        inference::Classifier().SetLabels(labels);
    }
    void SetNormalize(const float mean_vals[3],
                      const float norm_vals[3]) override
    {
        inference::Classifier().SetNormalize(mean_vals, norm_vals);
    }
    bool Init() override { return inference::Classifier().Init(); }
    std::string Infer(const cv::Mat &image, float &confidence) override
    {
        return inference::Classifier().Infer(image, confidence);
    }
};

class CameraAdapter final : public VisionCameraPort {
public:
    void start_collect() override { platform::Camera().start_collect(); }
    cv::Mat get_frame_raw() override
    {
        return platform::Camera().get_frame_raw();
    }
    bool is_cam_opened() const override
    {
        return platform::Camera().is_cam_opened();
    }
    void set_exposure_manual(int exposure) override
    {
        platform::Camera().set_exposure_manual(exposure);
    }
    int get_camera_width() const override
    {
        return platform::Camera().get_camera_width();
    }
    int get_camera_height() const override
    {
        return platform::Camera().get_camera_height();
    }
    int get_camera_fps() const override
    {
        return platform::Camera().get_camera_fps();
    }
};

class StreamAdapter final : public VisionStreamPort {
public:
    void start_server(int port) override
    {
        transport::CameraStreamServer().start_server(port);
    }
};

class DirectionControllerAdapter final : public VisionDirectionControllerPort {
public:
    explicit DirectionControllerAdapter(Direction_PID &controller)
        : VisionDirectionControllerPort(controller.Kp), controller_(controller) {}

    float Calculate(float expect, float feedback) override
    {
        return Image_PID_Calculate(&controller_, expect, feedback);
    }

private:
    Direction_PID &controller_;
};

}  // namespace

VisionClassifierPort &VisionClassifier()
{
    static ClassifierAdapter adapter;
    return adapter;
}

VisionCameraPort &VisionCamera()
{
    static CameraAdapter adapter;
    return adapter;
}

VisionStreamPort &VisionStream()
{
    static StreamAdapter adapter;
    return adapter;
}

const VisionParametersPort &VisionParameters()
{
    const auto &parameters = parameters::CurrentParameters();
    static const VisionParametersPort adapter{
        parameters.debug_rgb_r_min,
        parameters.debug_rgb_rb_diff,
        parameters.debug_rgb_rg_diff,
    };
    return adapter;
}

VisionDirectionControllerPort &VisionDirectionController()
{
    static DirectionControllerAdapter adapter{control::ImageController()};
    return adapter;
}

float CalculateVisionDirection(VisionDirectionControllerPort *controller,
                               float expect, float feedback)
{
    return controller->Calculate(expect, feedback);
}

float &MutableVisionImageOutput() { return control::MutableImageOutput(); }
float VisionMasterSpeed() { return control::MasterSpeed(); }
float VisionCurrentSpeed() { return control::CurrentSpeed(); }

const VisionEncoderPort &VisionLeftEncoder()
{
    static const VisionEncoderPort adapter{platform::LeftEncoder().count_now};
    return adapter;
}

const VisionEncoderPort &VisionRightEncoder()
{
    static const VisionEncoderPort adapter{platform::RightEncoder().count_now};
    return adapter;
}

void SetVisionEscDuty(int duty) { platform::SetEscDuty(duty); }
void SetVisionRunMode(int8_t value) { SetRunMode(value); }
int16_t VisionDistanceRaw() { return platform::DistanceRaw(); }

}  // namespace primer::port
