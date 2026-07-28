#include <cstdint>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#include "vision/bev/bev_image_segment_connectivity.hpp"

namespace {

struct Failure { std::string message; };

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw Failure{message};
    }
}

struct Frame {
    std::vector<std::uint8_t> bytes{};
    ls2k::port::CameraPixelFrameView view{};

    explicit Frame(std::uint8_t fill = 200U)
        : bytes(320U * 240U * 2U, 128U) {
        for (std::size_t index = 0U; index < bytes.size(); index += 2U) {
            bytes[index] = fill;
        }
        view.valid = true;
        view.format = ls2k::port::CameraFrameFormat::kYuyv;
        view.data = bytes.data();
        view.width = 320;
        view.height = 240;
        view.stride = 640;
    }

    void Set(int row, int col, std::uint8_t y) {
        bytes[static_cast<std::size_t>(row * view.stride + col * 2)] = y;
    }
};

ls2k::vision::BEVProjector IdentityProjector() {
    ls2k::port::BEVProjectorCalibration calibration{};
    calibration.valid = true;
    calibration.source_points = {{{0.0F, 0.0F},
                                  {0.0F, 319.0F},
                                  {239.0F, 0.0F},
                                  {239.0F, 319.0F}}};
    calibration.target_points = {{{0.0F, 0.0F},
                                  {0.0F, 319.0F},
                                  {239.0F, 0.0F},
                                  {239.0F, 319.0F}}};
    ls2k::vision::BEVProjector projector{};
    Expect(projector.Configure(calibration), "identity projector must configure");
    return projector;
}

ls2k::port::BinaryModelState Model(bool valid = true,
                                   int residual_threshold = 0) {
    ls2k::port::BinaryModelState model{};
    model.valid = valid;
    model.residual_threshold =
        (ls2k::port::kBinaryResidualLumaScale -
         model.illumination_weight) *
        residual_threshold;
    model.source = valid ? ls2k::port::BinaryModelSource::kCurrent
                         : ls2k::port::BinaryModelSource::kNone;
    model.illumination.fill(100U);
    return model;
}

ls2k::vision::BEVSegmentConnectivityResult Evaluate(
    const Frame& frame,
    const ls2k::vision::BEVProjector& projector,
    ls2k::port::BEVPoint from,
    ls2k::port::BEVPoint to,
    ls2k::vision::BEVSegmentVisibilityPolicy policy,
    const ls2k::port::BinaryModelState& model = Model()) {
    const ls2k::vision::BEVImageSegmentConnectivity query(
        frame.view, projector, model);
    return query.Evaluate(from, to, policy);
}

void TestWhiteDirections(const ls2k::vision::BEVProjector& projector) {
    const Frame frame{};
    const auto full = ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment;
    for (const auto& segment : std::vector<std::pair<ls2k::port::BEVPoint,
                                                     ls2k::port::BEVPoint>>{
             {{3.0F, 1.0F}, {3.0F, 6.0F}},
             {{1.0F, 3.0F}, {6.0F, 3.0F}},
             {{1.0F, 1.0F}, {6.0F, 6.0F}}}) {
        const auto result = Evaluate(frame, projector, segment.first, segment.second, full);
        Expect(result.status == ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
               "all-white horizontal, vertical, and diagonal segments must connect");
        Expect(result.sampled_point_count > 0U,
               "a connected segment must report sampled source pixels");
        Expect(!result.visible_segment_clipped,
               "fully visible segments must not report clipping");
    }
}

void TestSingleBlackPixelBlocks(const ls2k::vision::BEVProjector& projector) {
    Frame frame{};
    frame.Set(3, 4, 20U);
    const auto result = Evaluate(
        frame,
        projector,
        {3.0F, 1.0F},
        {3.0F, 6.0F},
        ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment);
    Expect(result.status == ls2k::vision::BEVSegmentConnectivityStatus::kBlocked,
           "one black pixel on the projected segment must block");
    Expect(result.sampled_point_count == 4U,
           "horizontal traversal must stop at the first black pixel");
}

void TestThresholdEqualityAndEndpoint(const ls2k::vision::BEVProjector& projector) {
    Frame frame{};
    frame.Set(3, 6, 100U);
    const auto result = Evaluate(
        frame,
        projector,
        {3.0F, 1.0F},
        {3.0F, 6.0F},
        ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment,
        Model(true, 100));
    Expect(result.status == ls2k::vision::BEVSegmentConnectivityStatus::kBlocked,
           "residual equal to the binary threshold, including an endpoint, must be black");
}

void TestCornerSupercover(const ls2k::vision::BEVProjector& projector) {
    Frame frame{};
    frame.Set(1, 2, 20U);
    const auto result = Evaluate(
        frame,
        projector,
        {1.0F, 1.0F},
        {4.0F, 4.0F},
        ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment);
    Expect(result.status == ls2k::vision::BEVSegmentConnectivityStatus::kBlocked,
           "supercover must inspect both cells touched at a pixel-corner tie");
}

void TestVisibilityPolicies(const ls2k::vision::BEVProjector& projector) {
    const Frame white{};
    const auto strict = Evaluate(
        white,
        projector,
        {-2.0F, 3.0F},
        {4.0F, 3.0F},
        ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment);
    Expect(strict.status == ls2k::vision::BEVSegmentConnectivityStatus::kUnobservable,
           "full-segment policy must reject an out-of-frame endpoint");
    Expect(strict.sampled_point_count == 0U,
           "strict visibility rejection must occur before sampling");

    const auto clipped = Evaluate(
        white,
        projector,
        {-2.0F, 3.0F},
        {4.0F, 3.0F},
        ls2k::vision::BEVSegmentVisibilityPolicy::kAllowFromEndpointClip);
    Expect(clipped.status == ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
           "the visible white suffix from an origin-like endpoint must connect");
    Expect(clipped.visible_segment_clipped,
           "allowed origin-side clipping must remain observable");

    Frame blocked{};
    blocked.Set(2, 3, 20U);
    const auto blocked_result = Evaluate(
        blocked,
        projector,
        {-2.0F, 3.0F},
        {4.0F, 3.0F},
        ls2k::vision::BEVSegmentVisibilityPolicy::kAllowFromEndpointClip);
    Expect(blocked_result.status == ls2k::vision::BEVSegmentConnectivityStatus::kBlocked,
           "a black pixel in the clipped visible suffix must block");
}

void TestInvalidThresholdAndProjection(const ls2k::vision::BEVProjector& projector) {
    const Frame frame{};
    const auto invalid_threshold = Evaluate(
        frame,
        projector,
        {1.0F, 1.0F},
        {4.0F, 4.0F},
        ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment,
        Model(false));
    Expect(invalid_threshold.status ==
               ls2k::vision::BEVSegmentConnectivityStatus::kUnobservable,
           "connectivity without a valid shared threshold must be unobservable");

    const float nan = std::numeric_limits<float>::quiet_NaN();
    const auto invalid_projection = Evaluate(
        frame,
        projector,
        {nan, 1.0F},
        {4.0F, 4.0F},
        ls2k::vision::BEVSegmentVisibilityPolicy::kRequireFullSegment);
    Expect(invalid_projection.status ==
               ls2k::vision::BEVSegmentConnectivityStatus::kUnobservable,
           "projection failure must be unobservable");
}

}  // namespace

int main() {
    try {
        const auto projector = IdentityProjector();
        TestWhiteDirections(projector);
        TestSingleBlackPixelBlocks(projector);
        TestThresholdEqualityAndEndpoint(projector);
        TestCornerSupercover(projector);
        TestVisibilityPolicies(projector);
        TestInvalidThresholdAndProjection(projector);
    } catch (const Failure& failure) {
        std::cerr << "FAIL: " << failure.message << '\n';
        return 1;
    }
    std::cout << "PASS: illumination binary BEV segment connectivity tests\n";
    return 0;
}
