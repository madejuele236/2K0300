#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "vision/bev/bev_image_segment_connectivity.hpp"

namespace {

struct TestFailure {
    std::string message;
};

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw TestFailure{message};
    }
}

struct TestFrame {
    std::vector<std::uint8_t> storage;
    ls2k::port::CameraPixelFrameView view{};

    explicit TestFrame(std::uint8_t fill = 100U) : storage(8U * 8U * 2U, 128U) {
        for (std::size_t i = 0; i < storage.size(); i += 2U) {
            storage[i] = fill;
        }
        view.valid = true;
        view.format = ls2k::port::CameraFrameFormat::kYuyv;
        view.data = storage.data();
        view.width = 8;
        view.height = 8;
        view.stride = 16;
    }

    void Set(int row, int col, std::uint8_t luma) {
        storage[static_cast<std::size_t>(row * view.stride + col * 2)] = luma;
    }
};

ls2k::vision::BEVProjector MakeIdentityProjector() {
    ls2k::port::BEVProjectorCalibration calibration{};
    calibration.valid = true;
    calibration.source_points = {{{0.0F, 0.0F},
                                  {0.0F, 7.0F},
                                  {7.0F, 0.0F},
                                  {7.0F, 7.0F}}};
    calibration.target_points = {{{0.0F, 0.0F},
                                  {0.0F, 7.0F},
                                  {7.0F, 0.0F},
                                  {7.0F, 7.0F}}};
    ls2k::vision::BEVProjector projector{};
    Expect(projector.Configure(calibration), "identity projector must configure");
    return projector;
}

ls2k::vision::BEVSegmentConnectivityResult Evaluate(
    const TestFrame& frame,
    const ls2k::vision::BEVProjector& projector,
    ls2k::port::BEVPoint from,
    ls2k::port::BEVPoint to,
    ls2k::vision::BEVSegmentVisibilityPolicy policy,
    int threshold = 30) {
    ls2k::port::BEVBoundaryParameters params{};
    params.local_jump_min_y = threshold;
    const ls2k::vision::BEVImageSegmentConnectivity query(frame.view, projector, params);
    return query.Evaluate(from, to, policy);
}

void ExpectConnectedDirectionCases(const ls2k::vision::BEVProjector& projector) {
    const TestFrame frame{};
    const auto policy = ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment;
    for (const auto& endpoints :
         std::vector<std::pair<ls2k::port::BEVPoint, ls2k::port::BEVPoint>>{
             {{2.0F, 1.0F}, {2.0F, 6.0F}},
             {{1.0F, 3.0F}, {6.0F, 3.0F}},
             {{1.0F, 1.0F}, {6.0F, 6.0F}}}) {
        const auto result = Evaluate(frame, projector, endpoints.first, endpoints.second, policy);
        Expect(result.status == ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
               "horizontal, vertical, and diagonal uniform segments must connect");
        Expect(result.sampled_point_count == 6U, "five-pixel span must sample six points");
        Expect(!result.visible_segment_clipped, "fully visible segment must not report clipping");
    }
}

void ExpectBlockedAtExactThreshold(const ls2k::vision::BEVProjector& projector) {
    TestFrame frame{};
    frame.Set(3, 4, 130U);
    const auto result = Evaluate(frame,
                                 projector,
                                 {3.0F, 1.0F},
                                 {3.0F, 6.0F},
                                 ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment,
                                 30);
    Expect(result.status == ls2k::vision::BEVSegmentConnectivityStatus::kBlocked,
           "delta equal to local_jump_min_y must block");
    Expect(result.sampled_point_count == 4U,
           "blocked result must count through the first threshold-crossing sample");
}

void ExpectStrictOutOfFrameUnobservable(const ls2k::vision::BEVProjector& projector) {
    const TestFrame frame{};
    const auto result = Evaluate(frame,
                                 projector,
                                 {-2.0F, 3.0F},
                                 {4.0F, 3.0F},
                                 ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment);
    Expect(result.status == ls2k::vision::BEVSegmentConnectivityStatus::kUnobservable,
           "strict policy must reject an out-of-frame endpoint");
    Expect(result.sampled_point_count == 0U, "rejected segment must not sample pixels");
    Expect(!result.visible_segment_clipped, "strict rejection is not a clipped observation");
}

void ExpectProjectionFailureUnobservable(const ls2k::vision::BEVProjector& projector) {
    const TestFrame frame{};
    const float invalid = std::numeric_limits<float>::quiet_NaN();
    const auto result = Evaluate(
        frame,
        projector,
        {invalid, 3.0F},
        {4.0F, 3.0F},
        ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment);
    Expect(result.status == ls2k::vision::BEVSegmentConnectivityStatus::kUnobservable,
           "projection failure must return unobservable");
    Expect(result.sampled_point_count == 0U,
           "projection failure must occur before pixel sampling");
    Expect(!result.visible_segment_clipped,
           "projection failure must not be reported as a clipped observation");
}

void ExpectFromEndpointClipCases(const ls2k::vision::BEVProjector& projector) {
    const auto policy = ls2k::vision::BEVSegmentVisibilityPolicy::kAllowFromEndpointClip;
    const TestFrame clear{};
    const auto clear_result = Evaluate(clear, projector, {-2.0F, 3.0F}, {4.0F, 3.0F}, policy);
    Expect(clear_result.status == ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
           "visible suffix from an origin-like outside endpoint must connect");
    Expect(clear_result.visible_segment_clipped, "allowed from-end clipping must be reported");
    Expect(clear_result.sampled_point_count == 5U, "clipped rows zero through four need five samples");

    TestFrame blocked{};
    blocked.Set(2, 3, 140U);
    const auto blocked_result = Evaluate(blocked, projector, {-2.0F, 3.0F}, {4.0F, 3.0F}, policy);
    Expect(blocked_result.status == ls2k::vision::BEVSegmentConnectivityStatus::kBlocked,
           "jump within clipped visible suffix must block");
    Expect(blocked_result.visible_segment_clipped, "blocked clipped suffix must retain clip fact");
}

}  // namespace

int main() {
    try {
        const ls2k::vision::BEVProjector projector = MakeIdentityProjector();
        ExpectConnectedDirectionCases(projector);
        ExpectBlockedAtExactThreshold(projector);
        ExpectStrictOutOfFrameUnobservable(projector);
        ExpectProjectionFailureUnobservable(projector);
        ExpectFromEndpointClipCases(projector);
    } catch (const TestFailure& failure) {
        std::cerr << "FAIL: " << failure.message << '\n';
        return 1;
    }
    std::cout << "PASS: BEV segment connectivity tests\n";
    return 0;
}
