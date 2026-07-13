#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include <charconv>
#include <cstddef>
#include <cstdint>
#include <iostream>

#include "platform/true_ls2k0300/camera_device.hpp"

namespace {

std::size_t OpenFdCount() {
    DIR* directory = opendir("/proc/self/fd");
    if (directory == nullptr) {
        return 0;
    }
    std::size_t count = 0;
    while (readdir(directory) != nullptr) {
        ++count;
    }
    closedir(directory);
    return count;
}

struct MemoryPages final {
    std::size_t virtual_pages = 0;
    std::size_t resident_pages = 0;
    bool valid = false;
};

MemoryPages ReadMemoryPages() {
    const int fd = open("/proc/self/statm", O_RDONLY | O_CLOEXEC);
    if (fd < 0) {
        return {};
    }
    char buffer[96]{};
    const ssize_t bytes = read(fd, buffer, sizeof(buffer));
    close(fd);
    if (bytes <= 0) {
        return {};
    }

    MemoryPages pages{};
    const char* begin = buffer;
    const char* end = buffer + bytes;
    const auto virtual_result = std::from_chars(begin, end, pages.virtual_pages);
    if (virtual_result.ec != std::errc{}) {
        return {};
    }
    begin = virtual_result.ptr;
    while (begin < end && (*begin == ' ' || *begin == '\t')) {
        ++begin;
    }
    const auto resident_result = std::from_chars(begin, end, pages.resident_pages);
    pages.valid = resident_result.ec == std::errc{};
    return pages;
}

}  // namespace

int main() {
    namespace platform = ls2k::platform::true_ls2k0300;

    const std::size_t initial_fds = OpenFdCount();
    platform::CameraDevice camera;
    platform::CameraConfig config{};
    config.device = "/dev/video0";
    config.width = 320;
    config.height = 240;
    config.fps = 60;
    config.buffer_count = 3;
    config.timeout_ms = 100;
    std::size_t stable_virtual_pages = 0;
    MemoryPages final_memory{};

    for (int cycle = 0; cycle < 3; ++cycle) {
        if (!camera.Start(config)) {
            std::cerr << "camera_board_test start failed cycle=" << cycle << '\n';
            return 1;
        }

        int valid_frames = 0;
        for (int attempt = 0; attempt < 30 && valid_frames < 5; ++attempt) {
            const platform::CameraFrameView frame = camera.Capture();
            if (!frame.valid()) {
                continue;
            }
            if (frame.width != config.width || frame.height != config.height ||
                frame.bytes < static_cast<std::size_t>(config.width * config.height * 2)) {
                std::cerr << "camera_board_test geometry mismatch cycle=" << cycle << '\n';
                return 2;
            }
            ++valid_frames;
        }
        if (valid_frames < 5) {
            std::cerr << "camera_board_test insufficient frames cycle=" << cycle << '\n';
            return 3;
        }

        camera.Stop();
        if (camera.Running()) {
            std::cerr << "camera_board_test still running cycle=" << cycle << '\n';
            return 4;
        }
        const std::size_t current_fds = OpenFdCount();
        if (initial_fds != 0 && current_fds != initial_fds) {
            std::cerr << "camera_board_test fd leak cycle=" << cycle
                      << " initial=" << initial_fds << " current=" << current_fds << '\n';
            return 5;
        }
        final_memory = ReadMemoryPages();
        if (!final_memory.valid) {
            std::cerr << "camera_board_test statm unavailable cycle=" << cycle << '\n';
            return 6;
        }
        if (cycle == 0) {
            stable_virtual_pages = final_memory.virtual_pages;
        } else if (final_memory.virtual_pages != stable_virtual_pages) {
            std::cerr << "camera_board_test virtual memory grew cycle=" << cycle
                      << " stable_pages=" << stable_virtual_pages
                      << " current_pages=" << final_memory.virtual_pages << '\n';
            return 7;
        }
    }

    std::cout << "camera_device_board_test passed cycles=3 frames_per_cycle=5 fd_count="
              << OpenFdCount() << '\n'
              << "camera_device_board_test memory virtual_pages_after_stop="
              << final_memory.virtual_pages
              << " resident_pages_after_stop=" << final_memory.resident_pages << '\n';
    return 0;
}
