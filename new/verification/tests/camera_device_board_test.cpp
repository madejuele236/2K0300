#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>

#include <charconv>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <string>

#include "platform/camera_v4l2_timestamp.hpp"
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

struct CaptureSummary final {
    std::uint64_t frames = 0;
    std::uint64_t multi_drain_frames = 0;
    int max_drain_count = 0;
    std::uint64_t timestamp_valid_frames = 0;
    std::int64_t capture_to_dequeue_delta_sum_ms = 0;
    std::int64_t min_capture_to_dequeue_delta_ms = 0;
    std::int64_t max_capture_to_dequeue_delta_ms = 0;
    bool have_capture_to_dequeue_delta = false;
    std::uint64_t poll_sum_us = 0;
    std::uint64_t max_poll_us = 0;
    std::uint64_t dequeue_sum_us = 0;
    std::uint64_t max_dequeue_us = 0;
    std::uint64_t sequence_advances = 0;
    std::uint64_t sequence_gap_observations = 0;
    std::uint64_t sequence_gap_total = 0;
    std::uint64_t sequence_resets = 0;
    std::uint64_t sequence_wraps = 0;
    std::uint64_t sequence_repeats = 0;
    std::uint32_t previous_sequence = 0;
    bool have_sequence = false;
    std::map<std::string, std::uint64_t> typed_failures;
};

std::int64_t CaptureToDequeueDeltaMs(
    std::uint64_t capture_time_ms,
    std::uint64_t dequeue_time_ms) noexcept {
    constexpr std::uint64_t kSignedMax =
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
    if (dequeue_time_ms >= capture_time_ms) {
        const std::uint64_t delta = dequeue_time_ms - capture_time_ms;
        return delta > kSignedMax ? std::numeric_limits<std::int64_t>::max()
                                  : static_cast<std::int64_t>(delta);
    }
    const std::uint64_t delta = capture_time_ms - dequeue_time_ms;
    return delta > kSignedMax ? std::numeric_limits<std::int64_t>::min()
                              : -static_cast<std::int64_t>(delta);
}

bool CaptureContractValid(bool drain_ready_buffers,
                          std::size_t mapped_buffer_count,
                          int drained_buffer_count,
                          std::uint64_t capture_time_ms,
                          std::uint64_t dequeue_time_ms,
                          bool v4l2_timestamp_valid) noexcept {
    if (mapped_buffer_count == 0 || drained_buffer_count < 1 ||
        static_cast<std::size_t>(drained_buffer_count) > mapped_buffer_count ||
        (!drain_ready_buffers && drained_buffer_count != 1) ||
        capture_time_ms == 0 || dequeue_time_ms == 0) {
        return false;
    }
    return v4l2_timestamp_valid
               ? ls2k::platform::IsV4l2CaptureTimestampPlausible(
                     capture_time_ms, dequeue_time_ms)
               : capture_time_ms == dequeue_time_ms;
}

int RunSelfTest() {
    int failures = 0;
    const auto expect = [&failures](const char* name, bool actual, bool expected) {
        const bool passed = actual == expected;
        std::cout << "event=self_test_case name=" << name
                  << " expected=" << (expected ? 1 : 0)
                  << " actual=" << (actual ? 1 : 0)
                  << " status=" << (passed ? "passed" : "failed") << '\n';
        if (!passed) {
            ++failures;
        }
    };
    expect("single_count_one", CaptureContractValid(false, 3, 1, 1000, 1001, true), true);
    expect("single_count_two", CaptureContractValid(false, 3, 2, 1000, 1001, true), false);
    expect("drain_within_actual_mapping",
           CaptureContractValid(true, 2, 2, 1000, 1001, true), true);
    expect("drain_above_actual_mapping",
           CaptureContractValid(true, 2, 3, 1000, 1001, true), false);
    expect("trusted_earlier", CaptureContractValid(true, 3, 1, 1000, 1100, true), true);
    expect("trusted_future_slack", CaptureContractValid(true, 3, 1, 1005, 1000, true), true);
    expect("trusted_beyond_future_slack",
           CaptureContractValid(true, 3, 1, 1006, 1000, true), false);
    expect("trusted_too_old", CaptureContractValid(true, 3, 1, 1000, 2001, true), false);
    expect("fallback_equal", CaptureContractValid(true, 3, 1, 1000, 1000, false), true);
    expect("fallback_unequal", CaptureContractValid(true, 3, 1, 1000, 1001, false), false);
    expect("zero_capture", CaptureContractValid(true, 3, 1, 0, 1000, true), false);
    expect("zero_dequeue", CaptureContractValid(true, 3, 1, 1000, 0, true), false);
    expect("signed_earlier_delta", CaptureToDequeueDeltaMs(1000, 1100) == 100, true);
    expect("signed_future_delta", CaptureToDequeueDeltaMs(1005, 1000) == -5, true);
    std::cout << "event=self_test_result status="
              << (failures == 0 ? "passed" : "failed")
              << " failures=" << failures << '\n';
    return failures == 0 ? 0 : 1;
}

void ObserveSequence(std::uint32_t sequence, CaptureSummary& summary) {
    if (!summary.have_sequence) {
        summary.previous_sequence = sequence;
        summary.have_sequence = true;
        return;
    }
    if (sequence == summary.previous_sequence) {
        ++summary.sequence_repeats;
    } else if (sequence > summary.previous_sequence) {
        ++summary.sequence_advances;
        const std::uint64_t delta =
            static_cast<std::uint64_t>(sequence) - summary.previous_sequence;
        if (delta > 1) {
            ++summary.sequence_gap_observations;
            summary.sequence_gap_total += delta - 1;
        }
    } else if (summary.previous_sequence >= 0xF0000000U && sequence <= 0x0FFFFFFFU) {
        ++summary.sequence_advances;
        ++summary.sequence_wraps;
        const std::uint64_t delta =
            (UINT64_C(1) << 32U) - summary.previous_sequence + sequence;
        if (delta > 1) {
            ++summary.sequence_gap_observations;
            summary.sequence_gap_total += delta - 1;
        }
    } else {
        ++summary.sequence_resets;
    }
    summary.previous_sequence = sequence;
}

void MergeSummary(const CaptureSummary& cycle, CaptureSummary& mode) {
    mode.frames += cycle.frames;
    mode.multi_drain_frames += cycle.multi_drain_frames;
    if (cycle.max_drain_count > mode.max_drain_count) {
        mode.max_drain_count = cycle.max_drain_count;
    }
    mode.timestamp_valid_frames += cycle.timestamp_valid_frames;
    mode.capture_to_dequeue_delta_sum_ms += cycle.capture_to_dequeue_delta_sum_ms;
    if (cycle.have_capture_to_dequeue_delta) {
        if (!mode.have_capture_to_dequeue_delta ||
            cycle.min_capture_to_dequeue_delta_ms <
                mode.min_capture_to_dequeue_delta_ms) {
            mode.min_capture_to_dequeue_delta_ms =
                cycle.min_capture_to_dequeue_delta_ms;
        }
        if (!mode.have_capture_to_dequeue_delta ||
            cycle.max_capture_to_dequeue_delta_ms >
                mode.max_capture_to_dequeue_delta_ms) {
            mode.max_capture_to_dequeue_delta_ms =
                cycle.max_capture_to_dequeue_delta_ms;
        }
        mode.have_capture_to_dequeue_delta = true;
    }
    mode.poll_sum_us += cycle.poll_sum_us;
    if (cycle.max_poll_us > mode.max_poll_us) {
        mode.max_poll_us = cycle.max_poll_us;
    }
    mode.dequeue_sum_us += cycle.dequeue_sum_us;
    if (cycle.max_dequeue_us > mode.max_dequeue_us) {
        mode.max_dequeue_us = cycle.max_dequeue_us;
    }
    mode.sequence_advances += cycle.sequence_advances;
    mode.sequence_gap_observations += cycle.sequence_gap_observations;
    mode.sequence_gap_total += cycle.sequence_gap_total;
    mode.sequence_resets += cycle.sequence_resets;
    mode.sequence_wraps += cycle.sequence_wraps;
    mode.sequence_repeats += cycle.sequence_repeats;
    for (const auto& failure : cycle.typed_failures) {
        mode.typed_failures[failure.first] += failure.second;
    }
}

void PrintSummary(const char* event,
                  const char* mode,
                  int cycle,
                  const CaptureSummary& summary) {
    const std::uint64_t denominator = summary.frames == 0 ? 1 : summary.frames;
    std::cout << "event=" << event << " mode=" << mode;
    if (cycle >= 0) {
        std::cout << " cycle=" << cycle;
    }
    std::cout << " frames=" << summary.frames
              << " multi_drain_frames=" << summary.multi_drain_frames
              << " max_drain_count=" << summary.max_drain_count
              << " timestamp_valid_frames=" << summary.timestamp_valid_frames
              << " avg_capture_to_dequeue_delta_ms="
              << summary.capture_to_dequeue_delta_sum_ms /
                     static_cast<std::int64_t>(denominator)
              << " min_capture_to_dequeue_delta_ms="
              << summary.min_capture_to_dequeue_delta_ms
              << " max_capture_to_dequeue_delta_ms="
              << summary.max_capture_to_dequeue_delta_ms
              << " avg_poll_wait_us=" << summary.poll_sum_us / denominator
              << " max_poll_wait_us=" << summary.max_poll_us
              << " avg_dequeue_us=" << summary.dequeue_sum_us / denominator
              << " max_dequeue_us=" << summary.max_dequeue_us
              << " sequence_advances=" << summary.sequence_advances
              << " sequence_gap_observations=" << summary.sequence_gap_observations
              << " sequence_gap_total=" << summary.sequence_gap_total
              << " sequence_resets=" << summary.sequence_resets
              << " sequence_wraps=" << summary.sequence_wraps
              << " sequence_repeats=" << summary.sequence_repeats
              << " typed_failure_count=";
    std::uint64_t failure_count = 0;
    for (const auto& failure : summary.typed_failures) {
        failure_count += failure.second;
    }
    std::cout << failure_count << " typed_failures=";
    if (summary.typed_failures.empty()) {
        std::cout << "none";
    } else {
        bool first = true;
        for (const auto& failure : summary.typed_failures) {
            if (!first) {
                std::cout << ',';
            }
            first = false;
            std::cout << failure.first << ':' << failure.second;
        }
    }
    std::cout << '\n';
}

}  // namespace

int main(int argc, char** argv) {
    namespace platform = ls2k::platform::true_ls2k0300;

    if (argc == 2 && std::strcmp(argv[1], "--self-test") == 0) {
        return RunSelfTest();
    }
    if (argc != 1) {
        std::cerr << "event=usage status=failed invocation=" << argv[0]
                  << " optional_arg=--self-test\n";
        return 64;
    }

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

    constexpr int kCyclesPerMode = 3;
    constexpr int kFramesPerCycle = 5;
    constexpr int kMaxAttemptsPerCycle = 30;
    int completed_cycles = 0;
    const bool drain_modes[] = {false, true};
    for (const bool drain_ready_buffers : drain_modes) {
        config.drain_ready_buffers = drain_ready_buffers;
        const char* mode = drain_ready_buffers ? "drain" : "single";
        CaptureSummary mode_summary{};
        std::cout << "event=config mode=" << mode
                  << " drain_ready_buffers=" << (drain_ready_buffers ? 1 : 0)
                  << " device=" << config.device
                  << " width=" << config.width
                  << " height=" << config.height
                  << " fps=" << config.fps
                  << " buffer_count=" << config.buffer_count
                  << " timeout_ms=" << config.timeout_ms
                  << " cycles=" << kCyclesPerMode
                  << " frames_per_cycle=" << kFramesPerCycle
                  << " max_attempts_per_cycle=" << kMaxAttemptsPerCycle << '\n';

        for (int cycle = 0; cycle < kCyclesPerMode; ++cycle) {
            const platform::CameraStartResult start = camera.Start(config);
            std::cout << "event=start mode=" << mode << " cycle=" << cycle
                      << " status=" << platform::CameraStartStatusCode(start.status)
                      << " stage=" << platform::CameraLifecycleStageCode(start.stage)
                      << " negotiated_width=" << start.negotiated.width
                      << " negotiated_height=" << start.negotiated.height
                      << " negotiated_stride=" << start.negotiated.stride
                      << " negotiated_pixel_format=" << start.negotiated.pixel_format
                      << " mapped_buffer_count=" << start.mapped_buffer_count << '\n';
            if (!start.ok()) {
                return 1;
            }

            CaptureSummary cycle_summary{};
            for (int attempt = 0;
                 attempt < kMaxAttemptsPerCycle &&
                 cycle_summary.frames < static_cast<std::uint64_t>(kFramesPerCycle);
                 ++attempt) {
                const platform::CameraCaptureResult capture = camera.Capture();
                if (!capture.ok()) {
                    const char* status = platform::CameraCaptureStatusCode(capture.status);
                    ++cycle_summary.typed_failures[status];
                    std::cout << "event=capture_failure mode=" << mode
                              << " cycle=" << cycle
                              << " attempt=" << attempt
                              << " status=" << status
                              << " stage=" << platform::CameraLifecycleStageCode(capture.stage)
                              << " drained_buffer_count=" << capture.drained_buffer_count
                              << " poll_wait_us=" << capture.poll_wait_us
                              << " dequeue_us=" << capture.dequeue_us << '\n';
                    continue;
                }

                const platform::CameraFrameView& frame = capture.frame;
                const std::size_t minimum_bytes =
                    static_cast<std::size_t>(frame.height) *
                    static_cast<std::size_t>(frame.stride);
                const bool valid_geometry = frame.valid() && frame.width == config.width &&
                    frame.height == config.height && frame.stride >= config.width * 2 &&
                    frame.bytes >= minimum_bytes;
                const bool valid_count = start.mapped_buffer_count != 0 &&
                    capture.drained_buffer_count >= 1 &&
                    static_cast<std::size_t>(capture.drained_buffer_count) <=
                        start.mapped_buffer_count &&
                    (drain_ready_buffers || capture.drained_buffer_count == 1);
                const bool valid_time = capture.v4l2_timestamp_valid
                    ? ls2k::platform::IsV4l2CaptureTimestampPlausible(
                          capture.capture_time_ms, capture.dequeue_time_ms)
                    : capture.capture_time_ms != 0 &&
                          capture.dequeue_time_ms != 0 &&
                          capture.capture_time_ms == capture.dequeue_time_ms;
                const bool valid_capture_contract = CaptureContractValid(
                    drain_ready_buffers,
                    start.mapped_buffer_count,
                    capture.drained_buffer_count,
                    capture.capture_time_ms,
                    capture.dequeue_time_ms,
                    capture.v4l2_timestamp_valid);
                const std::int64_t capture_to_dequeue_delta_ms =
                    CaptureToDequeueDeltaMs(
                        capture.capture_time_ms, capture.dequeue_time_ms);
                std::cout << "event=frame mode=" << mode
                          << " cycle=" << cycle
                          << " frame=" << cycle_summary.frames
                          << " drained_buffer_count=" << capture.drained_buffer_count
                          << " mapped_buffer_count=" << start.mapped_buffer_count
                          << " v4l2_sequence=" << capture.v4l2_sequence
                          << " capture_time_ms=" << capture.capture_time_ms
                          << " dequeue_time_ms=" << capture.dequeue_time_ms
                          << " capture_to_dequeue_delta_ms="
                          << capture_to_dequeue_delta_ms
                          << " v4l2_timestamp_valid="
                          << (capture.v4l2_timestamp_valid ? 1 : 0)
                          << " poll_wait_us=" << capture.poll_wait_us
                          << " dequeue_us=" << capture.dequeue_us
                          << " width=" << frame.width
                          << " height=" << frame.height
                          << " stride=" << frame.stride
                          << " bytes=" << frame.bytes
                          << " geometry_valid=" << (valid_geometry ? 1 : 0) << '\n';
                if (!valid_capture_contract || !valid_geometry) {
                    std::cerr << "event=invariant_failure mode=" << mode
                              << " cycle=" << cycle
                              << " frame=" << cycle_summary.frames
                              << " count_valid=" << (valid_count ? 1 : 0)
                              << " time_valid=" << (valid_time ? 1 : 0)
                              << " geometry_valid=" << (valid_geometry ? 1 : 0) << '\n';
                    return 2;
                }

                ++cycle_summary.frames;
                if (capture.drained_buffer_count > 1) {
                    ++cycle_summary.multi_drain_frames;
                }
                if (capture.drained_buffer_count > cycle_summary.max_drain_count) {
                    cycle_summary.max_drain_count = capture.drained_buffer_count;
                }
                if (capture.v4l2_timestamp_valid) {
                    ++cycle_summary.timestamp_valid_frames;
                }
                cycle_summary.capture_to_dequeue_delta_sum_ms +=
                    capture_to_dequeue_delta_ms;
                if (!cycle_summary.have_capture_to_dequeue_delta ||
                    capture_to_dequeue_delta_ms <
                        cycle_summary.min_capture_to_dequeue_delta_ms) {
                    cycle_summary.min_capture_to_dequeue_delta_ms =
                        capture_to_dequeue_delta_ms;
                }
                if (!cycle_summary.have_capture_to_dequeue_delta ||
                    capture_to_dequeue_delta_ms >
                        cycle_summary.max_capture_to_dequeue_delta_ms) {
                    cycle_summary.max_capture_to_dequeue_delta_ms =
                        capture_to_dequeue_delta_ms;
                }
                cycle_summary.have_capture_to_dequeue_delta = true;
                cycle_summary.poll_sum_us += capture.poll_wait_us;
                if (capture.poll_wait_us > cycle_summary.max_poll_us) {
                    cycle_summary.max_poll_us = capture.poll_wait_us;
                }
                cycle_summary.dequeue_sum_us += capture.dequeue_us;
                if (capture.dequeue_us > cycle_summary.max_dequeue_us) {
                    cycle_summary.max_dequeue_us = capture.dequeue_us;
                }
                ObserveSequence(capture.v4l2_sequence, cycle_summary);
            }
            if (cycle_summary.frames < static_cast<std::uint64_t>(kFramesPerCycle)) {
                PrintSummary("cycle_summary", mode, cycle, cycle_summary);
                std::cerr << "event=insufficient_frames mode=" << mode
                          << " cycle=" << cycle
                          << " required=" << kFramesPerCycle
                          << " actual=" << cycle_summary.frames << '\n';
                return 3;
            }

            camera.Stop();
            if (camera.Running()) {
                std::cerr << "event=lifecycle_failure mode=" << mode
                          << " cycle=" << cycle << " running_after_stop=1\n";
                return 4;
            }
            const std::size_t current_fds = OpenFdCount();
            if (initial_fds != 0 && current_fds != initial_fds) {
                std::cerr << "event=fd_leak mode=" << mode
                          << " cycle=" << cycle
                          << " initial_fds=" << initial_fds
                          << " current_fds=" << current_fds << '\n';
                return 5;
            }
            final_memory = ReadMemoryPages();
            if (!final_memory.valid) {
                std::cerr << "event=statm_unavailable mode=" << mode
                          << " cycle=" << cycle << '\n';
                return 6;
            }
            if (completed_cycles == 0) {
                stable_virtual_pages = final_memory.virtual_pages;
            } else if (final_memory.virtual_pages != stable_virtual_pages) {
                std::cerr << "event=virtual_memory_growth mode=" << mode
                          << " cycle=" << cycle
                          << " stable_pages=" << stable_virtual_pages
                          << " current_pages=" << final_memory.virtual_pages << '\n';
                return 7;
            }
            ++completed_cycles;
            std::cout << "event=lifecycle mode=" << mode
                      << " cycle=" << cycle
                      << " running_after_stop=0"
                      << " fd_count_after_stop=" << current_fds
                      << " virtual_pages_after_stop=" << final_memory.virtual_pages
                      << " resident_pages_after_stop=" << final_memory.resident_pages << '\n';
            PrintSummary("cycle_summary", mode, cycle, cycle_summary);
            MergeSummary(cycle_summary, mode_summary);
        }
        PrintSummary("mode_summary", mode, -1, mode_summary);
    }

    std::cout << "event=result status=passed modes=2 total_cycles=" << completed_cycles
              << " frames_per_cycle=" << kFramesPerCycle
              << " fd_count=" << OpenFdCount()
              << " virtual_pages_after_stop=" << final_memory.virtual_pages
              << " resident_pages_after_stop=" << final_memory.resident_pages << '\n';
    return 0;
}
