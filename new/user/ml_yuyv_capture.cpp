#include <charconv>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <limits>
#include <string_view>

#include "platform/true_ls2k0300/camera_device.hpp"

namespace {

bool ParseWarmupFrames(std::string_view text, int& value) {
    if (text.empty()) {
        return false;
    }
    int parsed = 0;
    const auto result = std::from_chars(text.data(), text.data() + text.size(), parsed);
    if (result.ec != std::errc{} || result.ptr != text.data() + text.size() || parsed < 0) {
        return false;
    }
    value = parsed;
    return true;
}

void PrintUsage(const char* program) {
    std::cerr << "usage: " << program << " OUTPUT_PATH WARMUP_FRAMES\n";
}

}  // namespace

int main(int argc, char** argv) {
    namespace platform = ls2k::platform::true_ls2k0300;

    if (argc != 3) {
        PrintUsage(argv[0]);
        return 2;
    }

    int warmup_frames = 0;
    if (!ParseWarmupFrames(argv[2], warmup_frames)) {
        std::cerr << "invalid WARMUP_FRAMES: expected a non-negative integer\n";
        PrintUsage(argv[0]);
        return 2;
    }

    platform::CameraConfig config{};
    platform::CameraDevice camera;
    if (!camera.Start(config)) {
        std::cerr << "camera start failed device=" << config.device
                  << " width=" << config.width
                  << " height=" << config.height
                  << " fps=" << config.fps << '\n';
        return 3;
    }

    for (int index = 0; index < warmup_frames; ++index) {
        if (!camera.Capture().valid()) {
            std::cerr << "camera warmup capture failed frame=" << (index + 1)
                      << " requested=" << warmup_frames << '\n';
            camera.Stop();
            return 4;
        }
    }

    const platform::CameraFrameView frame = camera.Capture();
    if (!frame.valid()) {
        std::cerr << "camera output capture failed after_warmup=" << warmup_frames << '\n';
        camera.Stop();
        return 5;
    }

    const std::size_t height = static_cast<std::size_t>(frame.height);
    const std::size_t stride = static_cast<std::size_t>(frame.stride);
    if (height != 0 && stride > std::numeric_limits<std::size_t>::max() / height) {
        std::cerr << "camera frame size overflow height=" << frame.height
                  << " stride=" << frame.stride << '\n';
        camera.Stop();
        return 6;
    }
    const std::size_t output_bytes = stride * height;
    if (frame.bytes < output_bytes) {
        std::cerr << "camera frame is shorter than stride*height bytes=" << frame.bytes
                  << " required=" << output_bytes << '\n';
        camera.Stop();
        return 6;
    }

    std::ofstream output(argv[1], std::ios::binary | std::ios::trunc);
    if (!output) {
        std::cerr << "output open failed path=" << argv[1] << '\n';
        camera.Stop();
        return 7;
    }
    output.write(reinterpret_cast<const char*>(frame.data),
                 static_cast<std::streamsize>(output_bytes));
    if (!output) {
        std::cerr << "output write failed path=" << argv[1]
                  << " bytes=" << output_bytes << '\n';
        output.close();
        camera.Stop();
        return 8;
    }
    output.close();
    if (!output) {
        std::cerr << "output close failed path=" << argv[1] << '\n';
        camera.Stop();
        return 9;
    }

    camera.Stop();
    if (camera.Running()) {
        std::cerr << "camera stop failed\n";
        return 10;
    }

    std::cout << "ml_yuyv_capture wrote path=" << argv[1]
              << " width=" << frame.width
              << " height=" << frame.height
              << " stride=" << frame.stride
              << " bytes=" << output_bytes
              << " warmup_frames=" << warmup_frames << '\n';
    return 0;
}
