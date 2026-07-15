#include <array>
#include <cstdint>
#include <iostream>
#include <optional>
#include <stdexcept>

#include "platform/camera_v4l2_timestamp.hpp"
#include "runtime/capture/camera_capture_worker.hpp"
#include "runtime/capture/camera_frame_store.hpp"

namespace {

void Expect(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void TestYuyvToGrayRespectsBytesperline() {
    std::array<std::uint8_t, 24> yuyv = {
        10, 1, 20, 2, 30, 3, 40, 4, 99, 99, 99, 99,
        50, 5, 60, 6, 70, 7, 80, 8, 88, 88, 88, 88,
    };
    ls2k::port::LegacyCameraFrame gray{};
    Expect(ls2k::runtime::YuyvToGray(yuyv.data(), 4, 2, 12, gray), "YUYV conversion failed");
    Expect(gray.width == 4 && gray.height == 2, "gray geometry mismatch");
    const std::array<std::uint8_t, 8> expected = {10, 20, 30, 40, 50, 60, 70, 80};
    for (std::size_t index = 0; index < expected.size(); ++index) {
        Expect(gray.gray[index] == expected[index], "gray sample mismatch");
    }

    ls2k::port::LegacyCameraFrame invalid{};
    Expect(!ls2k::runtime::YuyvToGray(yuyv.data(), 4, 2, 7, invalid),
           "short bytesperline must fail closed");
}

void TestV4l2TimestampSelectionTrustsOnlyMonotonicSameDomainTime() {
    timeval timestamp{};
    timestamp.tv_sec = 10;
    timestamp.tv_usec = 123000;
    const ls2k::platform::V4l2CaptureTimestampSelection trusted =
        ls2k::platform::SelectV4l2CaptureTimestamp(
            timestamp,
            V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
            10130);
    Expect(trusted.v4l2_timestamp_valid, "monotonic same-domain timestamp must be trusted");
    Expect(trusted.capture_time_ms == 10123, "trusted timestamp conversion mismatch");

    timeval future_slack_timestamp{};
    future_slack_timestamp.tv_sec = 10;
    future_slack_timestamp.tv_usec = 135000;
    const ls2k::platform::V4l2CaptureTimestampSelection future_slack =
        ls2k::platform::SelectV4l2CaptureTimestamp(
            future_slack_timestamp,
            V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
            10130);
    Expect(future_slack.v4l2_timestamp_valid,
           "timestamp within shared future slack must be trusted");
    Expect(future_slack.capture_time_ms == 10135,
           "future-slack timestamp conversion mismatch");

    const ls2k::platform::V4l2CaptureTimestampSelection unknown =
        ls2k::platform::SelectV4l2CaptureTimestamp(
            timestamp,
            V4L2_BUF_FLAG_TIMESTAMP_UNKNOWN,
            10130);
    Expect(!unknown.v4l2_timestamp_valid, "unknown timestamp kind must fall back");
    Expect(unknown.capture_time_ms == 10130, "unknown timestamp fallback mismatch");

    timeval future_timestamp{};
    future_timestamp.tv_sec = 11;
    future_timestamp.tv_usec = 0;
    const ls2k::platform::V4l2CaptureTimestampSelection future =
        ls2k::platform::SelectV4l2CaptureTimestamp(
            future_timestamp,
            V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
            10000);
    Expect(!future.v4l2_timestamp_valid, "future timestamp must fall back");
    Expect(future.capture_time_ms == 10000, "future timestamp fallback mismatch");

    timeval stale_timestamp{};
    stale_timestamp.tv_sec = 8;
    stale_timestamp.tv_usec = 0;
    const ls2k::platform::V4l2CaptureTimestampSelection stale =
        ls2k::platform::SelectV4l2CaptureTimestamp(
            stale_timestamp,
            V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC,
            10001);
    Expect(!stale.v4l2_timestamp_valid, "stale timestamp must fall back");
    Expect(stale.capture_time_ms == 10001, "stale timestamp fallback mismatch");
}

ls2k::port::LegacyCameraFrame MakeGrayFrame(std::uint8_t seed) {
    ls2k::port::LegacyCameraFrame frame{};
    frame.width = 2;
    frame.height = 2;
    frame.gray[0] = seed;
    frame.gray[1] = static_cast<std::uint8_t>(seed + 1U);
    frame.gray[2] = static_cast<std::uint8_t>(seed + 2U);
    frame.gray[3] = static_cast<std::uint8_t>(seed + 3U);
    return frame;
}

void TestFrameStoreLatestHistoryAndOverwrite() {
    ls2k::runtime::RuntimeState state{};
    ls2k::runtime::CameraFrameStore store(state);
    ls2k::port::CameraRawFrameMetadata metadata{};
    metadata.source = "unit_source";
    metadata.frame_id = 999;
    metadata.capture_time_ms = 777;
    metadata.dequeue_time_ms = 998;
    metadata.v4l2_sequence = 123;
    metadata.v4l2_timestamp_valid = true;
    metadata.drained_buffer_count = 2;
    metadata.poll_wait_us = 30;
    metadata.dequeue_us = 40;
    metadata.yuyv_to_gray_us = 50;

    ls2k::port::LegacyCameraFrame invalid_frame{};
    const ls2k::runtime::CameraFrameHandle invalid =
        store.Submit(invalid_frame.View(1, 100), metadata);
    Expect(!invalid.valid, "invalid frame must not submit");
    Expect(store.Health().dropped_frame_count == 1, "dropped counter mismatch");

    ls2k::port::LegacyCameraFrame first = MakeGrayFrame(10);
    const ls2k::runtime::CameraFrameHandle h1 = store.Submit(first.View(11, 1000), metadata);
    Expect(h1.valid, "first submit failed");
    Expect(store.LatestHandle().frame_id == 11, "latest handle mismatch");
    Expect(store.LatestHandle().metadata.source == "unit_source",
           "latest handle must preserve camera metadata source");
    Expect(store.LatestHandle().metadata.frame_id == 11,
           "store must align metadata frame id with submitted view");
    Expect(store.LatestHandle().metadata.capture_time_ms == 1000,
           "store must align metadata capture time with submitted view");
    Expect(store.LatestHandle().metadata.v4l2_sequence == 123,
           "latest handle must preserve v4l2 sequence");
    Expect(store.LatestHandle().metadata.v4l2_timestamp_valid,
           "latest handle must preserve v4l2 timestamp validity");
    Expect(store.LatestHandle().metadata.poll_wait_us == 30,
           "latest handle must preserve poll timing");
    Expect(store.LatestHandle().metadata.dequeue_us == 40,
           "latest handle must preserve dequeue timing");
    Expect(store.LatestHandle().metadata.yuyv_to_gray_us == 50,
           "latest handle must preserve conversion timing");
    Expect(store.TryGetLatestAfter(0).has_value(), "latest after old id missing");
    Expect(!store.TryGetLatestAfter(11).has_value(), "latest after same id must be empty");
    const std::optional<ls2k::runtime::CameraFrameHandle> history = store.FindExact(11, 1000);
    Expect(history.has_value(), "history exact lookup missing");
    Expect(history->metadata.source == "unit_source",
           "history handle must preserve camera metadata");

    ls2k::port::LegacyCameraFrame copied{};
    Expect(store.CopyFrame(h1, copied), "copy first frame failed");
    Expect(copied.gray[0] == 10 && copied.gray[3] == 13, "copied frame data mismatch");

    for (std::uint64_t id = 12; id <= 14; ++id) {
        ls2k::port::LegacyCameraFrame frame = MakeGrayFrame(static_cast<std::uint8_t>(id));
        const auto handle = store.Submit(frame.View(id, 1000 + id), metadata);
        Expect(handle.valid, "submit while rotating slots failed");
    }
    Expect(store.Health().submitted_frame_count == 4, "submitted counter mismatch");
    Expect(store.Health().overwritten_frame_count >= 1, "overwrite counter did not advance");
    Expect(store.LatestHandle().frame_id == 14, "latest did not advance");
    Expect(!store.CopyFrame(h1, copied), "overwritten generation must not copy");
}

void TestReadLeaseProtectsSlotUntilRelease() {
    ls2k::runtime::RuntimeState state{};
    ls2k::runtime::CameraFrameStore store(state);

    ls2k::port::LegacyCameraFrame first = MakeGrayFrame(20);
    const ls2k::runtime::CameraFrameHandle h1 = store.Submit(first.View(21, 2100), {});
    Expect(h1.valid, "read lease seed submit failed");

    std::optional<ls2k::runtime::CameraFrameStore::ReadLease> lease = store.Acquire(h1);
    Expect(lease.has_value(), "read lease acquire failed");
    Expect(lease->View().gray[0] == 20, "read lease view data mismatch");

    for (std::uint64_t id = 22; id <= 27; ++id) {
        ls2k::port::LegacyCameraFrame frame = MakeGrayFrame(static_cast<std::uint8_t>(id));
        const auto handle = store.Submit(frame.View(id, 2100 + id), {});
        Expect(handle.valid, "submit with one protected slot should use remaining slots");
    }

    ls2k::port::LegacyCameraFrame copied{};
    Expect(store.CopyFrame(h1, copied), "protected frame should remain copyable while leased");
    Expect(copied.gray[0] == 20, "protected frame pixels changed while leased");

    lease.reset();
    for (std::uint64_t id = 28; id <= 31; ++id) {
        ls2k::port::LegacyCameraFrame frame = MakeGrayFrame(static_cast<std::uint8_t>(id));
        (void)store.Submit(frame.View(id, 2100 + id), {});
    }
    Expect(!store.CopyFrame(h1, copied), "released frame should eventually be overwritable");
}

void TestWriteLeaseCommitAbortAndNoSlotDrop() {
    ls2k::runtime::RuntimeState state{};
    ls2k::runtime::CameraFrameStore store(state);

    auto lease = store.ReserveWritable(2, 2, {});
    Expect(lease.has_value(), "write lease reserve failed");
    ls2k::port::MutableLegacyCameraFrameView view = lease->MutableView();
    Expect(view.Valid(), "write lease mutable view invalid");
    view.gray[0] = 90;
    view.gray[1] = 91;
    view.gray[2] = 92;
    view.gray[3] = 93;
    ls2k::port::CameraRawFrameMetadata metadata{};
    metadata.source = "write_lease_test";
    const ls2k::runtime::CameraFrameHandle committed =
        lease->Commit(90, 9000, metadata);
    Expect(committed.valid, "write lease commit failed");
    Expect(store.LatestHandle().frame_id == 90, "write lease did not publish latest");
    ls2k::port::LegacyCameraFrame copied{};
    Expect(store.CopyFrame(committed, copied), "committed write lease frame should copy");
    Expect(copied.gray[0] == 90 && copied.gray[3] == 93, "committed write pixels mismatch");
    Expect(store.LatestHandle().metadata.source == "write_lease_test",
           "write lease metadata source mismatch");

    auto aborting = store.ReserveWritable(2, 2, {});
    Expect(aborting.has_value(), "abort lease reserve failed");
    aborting->Abort();
    Expect(store.LatestHandle().frame_id == 90, "aborted write lease must not publish latest");

    auto w1 = store.ReserveWritable(2, 2, {});
    auto w2 = store.ReserveWritable(2, 2, {});
    auto w3 = store.ReserveWritable(2, 2, {});
    Expect(w1.has_value() && w2.has_value() && w3.has_value(),
           "three write leases should reserve all slots");
    const uint64_t dropped_before = store.Health().dropped_frame_count;
    auto w4 = store.ReserveWritable(2, 2, {});
    Expect(!w4.has_value(), "fourth write lease should fail when all slots are encoding");
    Expect(store.Health().dropped_frame_count == dropped_before + 1,
           "no-slot write reserve should advance dropped counter");
}

void TestRawYuyvPixelViewPreservesStrideAndFormat() {
    ls2k::runtime::RuntimeState state{};
    ls2k::runtime::CameraFrameStore store(state);

    auto lease = store.ReserveWritable(ls2k::port::CameraFrameFormat::kYuyv,
                                       4,
                                       2,
                                       10,
                                       {});
    Expect(lease.has_value(), "raw yuyv write lease reserve failed");
    ls2k::port::MutableCameraPixelFrameView view = lease->MutablePixelView();
    Expect(view.Valid(), "raw yuyv mutable pixel view invalid");
    Expect(view.format == ls2k::port::CameraFrameFormat::kYuyv,
           "raw yuyv mutable view format mismatch");
    Expect(view.stride == 10, "raw yuyv mutable view stride mismatch");
    for (int row = 0; row < view.height; ++row) {
        for (int offset = 0; offset < view.stride; ++offset) {
            view.data[static_cast<std::size_t>(row) * static_cast<std::size_t>(view.stride) +
                      static_cast<std::size_t>(offset)] =
                static_cast<std::uint8_t>(10 + row * 20 + offset);
        }
    }
    ls2k::port::CameraRawFrameMetadata metadata{};
    metadata.source = "raw_yuyv_test";
    const ls2k::runtime::CameraFrameHandle handle = lease->Commit(200, 3000, metadata);
    Expect(handle.valid, "raw yuyv commit failed");
    Expect(handle.format == ls2k::port::CameraFrameFormat::kYuyv,
           "raw yuyv handle format mismatch");
    std::optional<ls2k::runtime::CameraFrameStore::ReadLease> read = store.Acquire(handle);
    Expect(read.has_value(), "raw yuyv read lease missing");
    const ls2k::port::CameraPixelFrameView pixel = read->PixelView();
    Expect(pixel.Valid(), "raw yuyv pixel view invalid");
    Expect(pixel.format == ls2k::port::CameraFrameFormat::kYuyv,
           "raw yuyv pixel view format mismatch");
    Expect(pixel.stride == 10, "raw yuyv pixel view stride mismatch");
    Expect(pixel.data[0] == 10 && pixel.data[10] == 30,
           "raw yuyv pixel bytes were not preserved");
    Expect(!read->View().Valid(), "legacy gray view must not expose raw yuyv as gray");
}

}  // namespace

int main() {
    try {
        TestYuyvToGrayRespectsBytesperline();
        TestV4l2TimestampSelectionTrustsOnlyMonotonicSameDomainTime();
        TestFrameStoreLatestHistoryAndOverwrite();
        TestReadLeaseProtectsSlotUntilRelease();
        TestWriteLeaseCommitAbortAndNoSlotDrop();
        TestRawYuyvPixelViewPreservesStrideAndFormat();
    } catch (const std::exception& error) {
        std::cerr << "camera_frame_store_test failed: " << error.what() << "\n";
        return 1;
    }
    std::cout << "camera_frame_store_test passed\n";
    return 0;
}
