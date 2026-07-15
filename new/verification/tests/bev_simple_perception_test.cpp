#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "vision/bev/bev_projector.hpp"
#include "vision/bev/bev_boundary_trace_clip.hpp"
#include "vision/bev/bev_image_segment_connectivity.hpp"
#include "vision/bev/bev_reference_path_builder.hpp"
#include "vision/bev/bev_segment_connectivity.hpp"
#include "vision/bev/bev_simple_perception.hpp"
#include "reference/reference_continuity.hpp"
#include "reference/reference_usability.hpp"
#include "vision/bev/single_boundary_offset.hpp"

namespace {

struct TestFailure {
    std::string message;
};

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw TestFailure{message};
    }
}

void ExpectNear(float actual, float expected, float tolerance, const std::string& message) {
    if (std::abs(actual - expected) > tolerance) {
        throw TestFailure{message};
    }
}

void SetPixel(ls2k::port::LegacyCameraFrame& frame, int row, int col, std::uint8_t value) {
    if (row < 0 || col < 0 || row >= frame.height || col >= frame.width) {
        return;
    }
    frame.gray[static_cast<std::size_t>(row) * static_cast<std::size_t>(frame.width) +
               static_cast<std::size_t>(col)] = value;
}

void DrawPatch(ls2k::port::LegacyCameraFrame& frame, float row_px, float col_px, std::uint8_t value) {
    const int row = static_cast<int>(std::lround(row_px));
    const int col = static_cast<int>(std::lround(col_px));
    for (int dr = -2; dr <= 2; ++dr) {
        for (int dc = -2; dc <= 2; ++dc) {
            SetPixel(frame, row + dr, col + dc, value);
        }
    }
}

ls2k::port::LegacyCameraFrame MakeFrame(std::uint8_t fill = 0U) {
    ls2k::port::LegacyCameraFrame frame{};
    frame.width = ls2k::port::kCompiledCameraFrameWidth;
    frame.height = ls2k::port::kCompiledCameraFrameHeight;
    frame.gray.fill(fill);
    return frame;
}

struct TestPixelFrame {
    std::vector<std::uint8_t> bytes{};
    ls2k::port::CameraPixelFrameView view{};
};

TestPixelFrame MakeYuyvPixelFrame(const ls2k::port::LegacyCameraFrame& frame,
                                  std::uint64_t frame_id,
                                  std::uint64_t capture_time_ms) {
    TestPixelFrame pixel{};
    const int stride = frame.width * 2;
    pixel.bytes.assign(static_cast<std::size_t>(stride) *
                           static_cast<std::size_t>(frame.height),
                       128U);
    for (int row = 0; row < frame.height; ++row) {
        for (int col = 0; col < frame.width; ++col) {
            pixel.bytes[static_cast<std::size_t>(row) * static_cast<std::size_t>(stride) +
                        static_cast<std::size_t>(col) * 2U] =
                frame.gray[static_cast<std::size_t>(row) *
                               static_cast<std::size_t>(frame.width) +
                           static_cast<std::size_t>(col)];
        }
    }
    pixel.view.valid = true;
    pixel.view.format = ls2k::port::CameraFrameFormat::kYuyv;
    pixel.view.data = pixel.bytes.data();
    pixel.view.width = frame.width;
    pixel.view.height = frame.height;
    pixel.view.stride = stride;
    pixel.view.frame_id = frame_id;
    pixel.view.capture_time_ms = capture_time_ms;
    return pixel;
}

ls2k::vision::BEVProjector MakeProjector(const ls2k::port::RuntimeParameters& params) {
    ls2k::vision::BEVProjector projector{};
    Expect(projector.Configure(params.bev_projector), "default projector must configure");
    return projector;
}

void DrawVehicleStripe(ls2k::port::LegacyCameraFrame& frame,
                       const ls2k::vision::BEVProjector& projector,
                       const ls2k::port::RuntimeParameters& params,
                       float lateral_min,
                       float lateral_max,
                       std::uint8_t value) {
    const float lateral_step = std::max(0.01F, params.bev_geometry.lateral_step_m);
    const float forward_step =
        (params.bev_geometry.forward_samples_m[1] -
         params.bev_geometry.forward_samples_m[0]) /
        64.0F;
    const float forward_max = params.bev_geometry.forward_samples_m.back();
    for (float forward = 0.0F;
         forward <= forward_max + forward_step * 0.5F;
         forward += forward_step) {
        for (float lateral = -params.bev_geometry.search_lateral_limit_m;
             lateral <= params.bev_geometry.search_lateral_limit_m + lateral_step * 0.5F;
             lateral += lateral_step) {
            if (lateral < lateral_min || lateral > lateral_max) {
                continue;
            }
            ls2k::port::ImagePoint image{};
            if (projector.ProjectVehicleToImage({forward, lateral}, image)) {
                DrawPatch(frame, image.row_px, image.col_px, value);
            }
        }
    }
}

int CountPresentPathPoints(const ls2k::port::BEVReferencePath& reference,
                           ls2k::port::BEVPathPointSource source) {
    int count = 0;
    for (const ls2k::port::BEVPathSample& sample : reference.sampled_path) {
        if (sample.present && sample.source == source) {
            ++count;
        }
    }
    return count;
}

ls2k::vision::BEVSimpleRowScan SyntheticRow(float forward_m,
                                            float sampleable_low_m,
                                            float sampleable_high_m) {
    ls2k::vision::BEVSimpleRowScan row{};
    row.valid = true;
    row.forward_m = forward_m;
    row.sampleable_count = 101;
    row.sampleable_left_m = sampleable_low_m;
    row.sampleable_right_m = sampleable_high_m;
    row.sampleable_width_m = sampleable_high_m - sampleable_low_m;
    return row;
}

int SyntheticLateralIndex(const ls2k::vision::BEVSimpleRowScan& row,
                          float lateral_m) {
    const float offset_m = lateral_m - row.sampleable_left_m;
    return static_cast<int>(std::lround(offset_m * 1000.0F));
}

void AddSyntheticInterval(ls2k::vision::BEVSimpleRowScan& row,
                          float low_m,
                          float high_m) {
    const bool have_sampleable_bounds = row.sampleable_width_m > 0.0F;
    const bool low_edge_visible =
        !have_sampleable_bounds || low_m > row.sampleable_left_m + 1.0e-4F;
    const bool high_edge_visible =
        !have_sampleable_bounds || high_m < row.sampleable_right_m - 1.0e-4F;
    const int low_index = SyntheticLateralIndex(row, low_m);
    const int high_index = SyntheticLateralIndex(row, high_m);
    if (low_edge_visible && high_edge_visible) {
        ls2k::vision::BEVBoundarySpan span{};
        span.forward_m = row.forward_m;
        span.left_m = low_m;
        span.right_m = high_m;
        span.center_m = 0.5F * (low_m + high_m);
        span.width_m = high_m - low_m;
        span.left_lateral_index = low_index;
        span.right_lateral_index = high_index;
        row.spans.push_back(span);
    }

    if (low_edge_visible) {
        ls2k::vision::BEVBoundaryJump left_jump{};
        left_jump.forward_m = row.forward_m;
        left_jump.lateral_m = low_m;
        left_jump.lateral_index = low_index;
        left_jump.delta_y = 80;
        left_jump.polarity = ls2k::vision::BEVBoundaryJumpPolarity::kRisingY;
        row.jumps.push_back(left_jump);
    }

    if (high_edge_visible) {
        ls2k::vision::BEVBoundaryJump right_jump{};
        right_jump.forward_m = row.forward_m;
        right_jump.lateral_m = high_m;
        right_jump.lateral_index = high_index;
        right_jump.delta_y = -80;
        right_jump.polarity = ls2k::vision::BEVBoundaryJumpPolarity::kFallingY;
        row.jumps.push_back(right_jump);
    }
}

class AlwaysConnectedQuery final
    : public ls2k::vision::BEVSegmentConnectivityQuery {
public:
    ls2k::vision::BEVSegmentConnectivityResult Evaluate(
        const ls2k::port::BEVPoint&,
        const ls2k::port::BEVPoint&,
        ls2k::vision::BEVSegmentVisibilityPolicy) const override {
        return {ls2k::vision::BEVSegmentConnectivityStatus::kConnected, 1U, false};
    }
};

class ScriptedConnectivityQuery final
    : public ls2k::vision::BEVSegmentConnectivityQuery {
public:
    explicit ScriptedConnectivityQuery(
        std::vector<ls2k::vision::BEVSegmentConnectivityStatus> statuses)
        : statuses_(std::move(statuses)) {}

    ls2k::vision::BEVSegmentConnectivityResult Evaluate(
        const ls2k::port::BEVPoint& from,
        const ls2k::port::BEVPoint& to,
        ls2k::vision::BEVSegmentVisibilityPolicy policy) const override {
        from_points.push_back(from);
        to_points.push_back(to);
        policies.push_back(policy);
        const std::size_t index = from_points.size() - 1U;
        const ls2k::vision::BEVSegmentConnectivityStatus status =
            index < statuses_.size()
                ? statuses_[index]
                : ls2k::vision::BEVSegmentConnectivityStatus::kConnected;
        return {status, 1U, false};
    }

    mutable std::vector<ls2k::port::BEVPoint> from_points{};
    mutable std::vector<ls2k::port::BEVPoint> to_points{};
    mutable std::vector<ls2k::vision::BEVSegmentVisibilityPolicy> policies{};

private:
    std::vector<ls2k::vision::BEVSegmentConnectivityStatus> statuses_;
};

ls2k::port::BEVReferencePath BuildSyntheticReferencePath(
    const std::vector<ls2k::vision::BEVSimpleRowScan>& rows,
    const ls2k::port::RuntimeParameters& params) {
    const AlwaysConnectedQuery query{};
    return ls2k::vision::BuildReferencePath(rows, params, query);
}

void TestConnectedPathSkipsFailuresAndPreservesPredecessor() {
    using ls2k::vision::BEVSegmentVisibilityPolicy;
    using ls2k::vision::BEVSegmentConnectivityStatus;

    ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    for (std::size_t index = 0U; index < 5U; ++index) {
        rows.push_back(SyntheticRow(
            params.bev_geometry.forward_samples_m[index], -1.0F, 1.0F));
        AddSyntheticInterval(rows.back(), -0.20F, 0.20F);
    }
    ScriptedConnectivityQuery query({
        BEVSegmentConnectivityStatus::kBlocked,
        BEVSegmentConnectivityStatus::kConnected,
        BEVSegmentConnectivityStatus::kUnobservable,
        BEVSegmentConnectivityStatus::kBlocked,
        BEVSegmentConnectivityStatus::kConnected,
    });
    const ls2k::port::BEVReferencePath reference =
        ls2k::vision::BuildConnectedReferencePath(rows, params, query);

    Expect(CountPresentPathPoints(reference,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 2,
           "blocked and unobservable rows must be skipped while later connected rows reconnect");
    ExpectNear(reference.sampled_path[0].point.forward_m, rows[1].forward_m, 1.0e-6F,
               "leading rejected row must compact the first accepted sample with true forward");
    ExpectNear(reference.sampled_path[1].point.forward_m, rows[4].forward_m, 1.0e-6F,
               "reconnected sample after a gap must retain its true forward");
    ExpectNear(query.from_points[0].forward_m, 0.0F, 1.0e-6F,
               "first candidate must be tested from the vehicle origin");
    ExpectNear(query.from_points[1].forward_m, 0.0F, 1.0e-6F,
               "a rejected leading candidate must not update the origin predecessor");
    Expect(query.policies[0] == BEVSegmentVisibilityPolicy::kAllowFromEndpointClip &&
               query.policies[1] == BEVSegmentVisibilityPolicy::kAllowFromEndpointClip,
           "all candidates before the first acceptance must allow origin endpoint clipping");
    ExpectNear(query.from_points[2].forward_m, rows[1].forward_m, 1.0e-6F,
               "post-acceptance checks must start from the last accepted sample");
    ExpectNear(query.from_points[4].forward_m, rows[1].forward_m, 1.0e-6F,
               "blocked and unobservable gaps must not update the predecessor");
    Expect(query.policies[2] == BEVSegmentVisibilityPolicy::kRequireFullSegment &&
               query.policies[4] == BEVSegmentVisibilityPolicy::kRequireFullSegment,
           "all checks after first acceptance must require the full segment");
}

void TestConnectedPathAcceptsFirstPassingCandidateInProductionOrder() {
    using ls2k::vision::BEVSegmentConnectivityStatus;

    ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    for (std::size_t index = 0U; index < 3U; ++index) {
        rows.push_back(SyntheticRow(
            params.bev_geometry.forward_samples_m[index], -1.0F, 1.0F));
        AddSyntheticInterval(rows.back(), -0.60F, -0.40F);
        AddSyntheticInterval(rows.back(), 0.30F, 0.50F);
    }
    ScriptedConnectivityQuery query({BEVSegmentConnectivityStatus::kBlocked,
                                     BEVSegmentConnectivityStatus::kConnected});
    const ls2k::port::BEVReferencePath reference =
        ls2k::vision::BuildConnectedReferencePath(rows, params, query);

    ExpectNear(query.to_points[0].lateral_m, -0.50F, 1.0e-5F,
               "candidate inspection must retain existing row production order");
    ExpectNear(query.to_points[1].lateral_m, 0.40F, 1.0e-5F,
               "connectivity must inspect the next candidate after a rejection");
    ExpectNear(reference.sampled_path[0].point.lateral_m, 0.40F, 1.0e-5F,
               "the first connected candidate in production order must be accepted");
}

void TestSingleBoundaryOffsetHelperGeometry() {
    {
        const std::vector<ls2k::port::BEVPoint> trace{{0.0F, 0.0F}, {1.0F, 0.0F}};
        const std::vector<float> targets{0.0F, 0.5F, 1.0F};
        const std::vector<ls2k::port::BEVPoint> result =
            ls2k::vision::BuildSingleBoundaryOffsetReference(trace, targets, 0.2F);
        Expect(result.size() == 3U, "straight helper trace must cover all target samples");
        ExpectNear(result[1].lateral_m, 0.2F, 1.0e-5F,
                   "zero-slope helper offset must equal signed lateral offset");
    }
    {
        const std::vector<ls2k::port::BEVPoint> trace{{0.0F, 0.0F}, {1.0F, 1.0F}};
        const std::vector<float> targets{0.5F};
        const std::vector<ls2k::port::BEVPoint> positive =
            ls2k::vision::BuildSingleBoundaryOffsetReference(trace, targets, 0.2F);
        const std::vector<ls2k::port::BEVPoint> negative =
            ls2k::vision::BuildSingleBoundaryOffsetReference(trace, targets, -0.2F);
        const std::vector<ls2k::port::BEVPoint> zero =
            ls2k::vision::BuildSingleBoundaryOffsetReference(trace, targets, 0.0F);
        const float expected_delta = 0.2F * std::sqrt(2.0F);
        Expect(positive.size() == 1U && negative.size() == 1U && zero.size() == 1U,
               "nonzero-slope helper trace must produce target sample");
        ExpectNear(positive[0].lateral_m, 0.5F + expected_delta, 1.0e-5F,
                   "positive helper offset must include local slope normal distance");
        ExpectNear(negative[0].lateral_m, 0.5F - expected_delta, 1.0e-5F,
                   "negative helper offset must include local slope normal distance");
        ExpectNear(zero[0].lateral_m, 0.5F, 1.0e-5F,
                   "zero helper offset must hug the boundary");
    }
    {
        const std::vector<ls2k::port::BEVPoint> trace{{0.0F, 0.0F}, {1.0F, 0.0F}};
        const std::vector<float> targets{0.0F, 0.5F, 1.5F};
        const std::vector<ls2k::port::BEVPoint> result =
            ls2k::vision::BuildSingleBoundaryOffsetReference(trace, targets, 0.2F);
        Expect(result.size() == 2U, "helper must stop leading output when target leaves trace range");
    }
    {
        const std::vector<float> targets{0.0F};
        const std::vector<ls2k::port::BEVPoint> one_point{{0.0F, 0.0F}};
        const std::vector<ls2k::port::BEVPoint> duplicate_y{
            {0.0F, 0.0F},
            {0.0F, 0.2F},
        };
        Expect(ls2k::vision::BuildSingleBoundaryOffsetReference(one_point, targets, 0.2F).empty(),
               "helper must reject insufficient trace points");
        Expect(ls2k::vision::BuildSingleBoundaryOffsetReference(duplicate_y, targets, 0.2F).empty(),
               "helper must reject non-single-valued trace points");
    }
}

void TestBoundaryTraceClipHelperRule() {
    using ls2k::vision::BEVBoundaryTraceClipOptions;
    using ls2k::vision::BEVBoundaryTracePoint;
    using ls2k::vision::ClipBoundaryTraceOutliers;

    {
        const std::vector<BEVBoundaryTracePoint> raw{
            {0U, {0.0F, 0.0F}},
            {1U, {0.1F, 0.1F}},
            {2U, {0.2F, 0.2F}},
        };
        const std::vector<BEVBoundaryTracePoint> clipped =
            ClipBoundaryTraceOutliers(raw, BEVBoundaryTraceClipOptions{0.2F});
        Expect(clipped.size() == 3U, "continuous boundary trace must keep all points");
    }
    {
        const std::vector<BEVBoundaryTracePoint> raw{
            {0U, {0.0F, 0.0F}},
            {1U, {0.0F, 1.0F}},
            {2U, {0.1F, 0.1F}},
        };
        const std::vector<BEVBoundaryTracePoint> clipped =
            ClipBoundaryTraceOutliers(raw, BEVBoundaryTraceClipOptions{0.2F});
        Expect(clipped.size() == 2U,
               "single outlier must be deleted without truncating later trace");
        Expect(clipped[0].row_index == 0U && clipped[1].row_index == 2U,
               "later point must be evaluated against the last kept point");
    }
    {
        const std::vector<BEVBoundaryTracePoint> raw{
            {0U, {0.0F, 0.0F}},
            {1U, {0.0F, 1.0F}},
            {2U, {0.0F, 1.1F}},
            {3U, {0.0F, 0.2F}},
        };
        const std::vector<BEVBoundaryTracePoint> clipped =
            ClipBoundaryTraceOutliers(raw, BEVBoundaryTraceClipOptions{0.2F});
        Expect(clipped.size() == 2U,
               "consecutive outliers must not replace the last kept point");
        Expect(clipped[0].row_index == 0U && clipped[1].row_index == 3U,
               "helper must preserve original order of kept points");
    }
}

void TestBevLocalBoundaryFacts() {
    ls2k::port::RuntimeParameters params{};
    ls2k::vision::BEVProjector projector = MakeProjector(params);
    ls2k::port::LegacyCameraFrame frame = MakeFrame(0U);
    DrawVehicleStripe(frame, projector, params, -0.18F, 0.18F, 255U);

    ls2k::vision::BEVSampleProjectionLut lut{};
    TestPixelFrame pixel = MakeYuyvPixelFrame(frame, 1U, 1U);
    const ls2k::vision::BEVSimplePerceptionResult result =
        ls2k::vision::RunBEVSimplePerception(pixel.view, params, projector, &lut);

    Expect(result.rows.size() == ls2k::port::kBevReferenceSampleCount,
           "row scanner must scan the configured BEV forward samples");

    bool saw_span = false;
    bool saw_jump_pair = false;
    bool saw_row_support_stats = false;
    for (const ls2k::vision::BEVSimpleRowScan& row : result.rows) {
        saw_span = saw_span || !row.spans.empty();
        saw_jump_pair = saw_jump_pair || row.jumps.size() >= 2U;
        saw_row_support_stats = saw_row_support_stats ||
                                (row.sampleable_count > 0U &&
                                 row.sampleable_width_m > 0.0F);
    }
    Expect(saw_span, "drawn BEV stripe must expose boundary span facts");
    Expect(saw_jump_pair, "drawn BEV stripe must expose local Y boundary jumps");
    Expect(saw_row_support_stats,
           "row scanner must expose sample support stats without changing reference facts");
    const ls2k::port::BEVReferencePath unconstrained_reference =
        BuildSyntheticReferencePath(result.rows, params);
    Expect(CountPresentPathPoints(unconstrained_reference,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) >= 3,
           "continuous row spans must first produce ordinary center candidates");
    const ls2k::port::ReferenceUsability usability =
        ls2k::reference::EvaluateReferenceUsability(result.reference_path, params);
    Expect(usability.usable,
           "continuous boundary spans must produce usable current facts: reason=" +
               usability.reason + " leading=" +
               std::to_string(usability.leading_usable_samples));
    Expect(CountPresentPathPoints(result.reference_path,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) >= 3,
           "reference points must explicitly come from boundary span centers");
}

void TestBevGeometryControlsWideImageScan() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.search_lateral_limit_m = 0.85F;
    ls2k::vision::BEVProjector projector = MakeProjector(params);
    ls2k::port::LegacyCameraFrame frame = MakeFrame(0U);
    DrawVehicleStripe(frame, projector, params, 0.50F, 0.70F, 255U);

    ls2k::vision::BEVSampleProjectionLut lut{};
    TestPixelFrame pixel = MakeYuyvPixelFrame(frame, 1U, 1U);
    const ls2k::vision::BEVSimplePerceptionResult result =
        ls2k::vision::RunBEVSimplePerception(pixel.view, params, projector, &lut);

    bool saw_wide_right_span = false;
    for (const ls2k::vision::BEVSimpleRowScan& row : result.rows) {
        for (const ls2k::vision::BEVBoundarySpan& span : row.spans) {
            saw_wide_right_span = saw_wide_right_span || span.center_m > 0.45F;
        }
    }
    Expect(saw_wide_right_span,
           "BEV row scanning must use the configured BEV image extent");
}

void TestHoldIsExplicitNonVisualSource() {
    ls2k::port::RuntimeParameters params{};
    ls2k::vision::BEVProjector projector = MakeProjector(params);
    ls2k::port::LegacyCameraFrame frame = MakeFrame(0U);
    DrawVehicleStripe(frame, projector, params, -0.18F, 0.18F, 255U);

    ls2k::vision::BEVSampleProjectionLut lut{};
    TestPixelFrame pixel = MakeYuyvPixelFrame(frame, 1U, 1U);
    const ls2k::vision::BEVSimplePerceptionResult first =
        ls2k::vision::RunBEVSimplePerception(pixel.view, params, projector, &lut);
    const ls2k::port::ReferenceUsability first_usability =
        ls2k::reference::EvaluateReferenceUsability(first.reference_path, params);
    Expect(first_usability.usable, "first frame must produce usable visual facts");
    const ls2k::port::ReferenceHoldState first_hold =
        ls2k::reference::MakeReferenceHoldState(first.reference_path, params);

    ls2k::port::LegacyCameraFrame blank = MakeFrame(0U);
    TestPixelFrame blank_pixel = MakeYuyvPixelFrame(blank, 2U, 2U);
    const ls2k::vision::BEVSimplePerceptionResult blank_facts =
        ls2k::vision::RunBEVSimplePerception(blank_pixel.view, params, projector, &lut);
    const ls2k::port::ReferenceUsability blank_usability =
        ls2k::reference::EvaluateReferenceUsability(blank_facts.reference_path, params);
    Expect(!blank_usability.usable, "blank current frame must be selected only if hold is unavailable");
    const ls2k::port::ReferenceContinuityResult held =
        ls2k::reference::BuildReferenceHoldCandidate(first_hold, params);

    Expect(held.hold_selected, "blank frame may explicitly hold the previous reference");
    Expect(ls2k::reference::EvaluateReferenceUsability(held.reference_path, params).usable,
           "held reference facts must still pass selected usability");
    Expect(held.reference_path.mode == ls2k::port::ReferenceMode::kHoldLast,
           "held reference must use hold mode");
    Expect(CountPresentPathPoints(held.reference_path, ls2k::port::BEVPathPointSource::kHold) >= 3,
           "held points must be marked as hold, not current-frame visual evidence");
    Expect(first_hold.last_reference[0].source == ls2k::port::BEVPathPointSource::kIntervalCenter,
           "hold output must not overwrite visual memory with hold source");

    ls2k::port::ReferenceHoldState source_free_memory = first_hold;
    for (ls2k::port::BEVPathSample& sample : source_free_memory.last_reference) {
        if (sample.present) {
            sample.source = ls2k::port::BEVPathPointSource::kNone;
        }
    }
    const ls2k::port::ReferenceContinuityResult held_from_geometry =
        ls2k::reference::BuildReferenceHoldCandidate(source_free_memory, params);
    Expect(held_from_geometry.hold_selected,
           "hold continuity must use present finite geometry, not source metadata");
    Expect(CountPresentPathPoints(held_from_geometry.reference_path,
                                  ls2k::port::BEVPathPointSource::kHold) >= 3,
           "hold output must rewrite source metadata to hold");

    ls2k::port::ReferenceHoldState invalid_geometry_memory = first_hold;
    invalid_geometry_memory.last_reference[0].point.lateral_m =
        std::numeric_limits<float>::quiet_NaN();
    const ls2k::port::ReferenceContinuityResult rejected_invalid_geometry =
        ls2k::reference::BuildReferenceHoldCandidate(invalid_geometry_memory, params);
    Expect(!rejected_invalid_geometry.hold_selected,
           "hold continuity must reject non-finite leading geometry");

    ls2k::port::RuntimeParameters changed_geometry = params;
    changed_geometry.bev_geometry.lateral_step_m *= 0.5F;
    const ls2k::port::ReferenceContinuityResult rejected =
        ls2k::reference::BuildReferenceHoldCandidate(first_hold, changed_geometry);
    Expect(!rejected.hold_selected, "geometry identity change must reject hold output");

    ls2k::port::RuntimeParameters changed_sparse_rows = params;
    changed_sparse_rows.bev_geometry.sparse_row_count = 12;
    const ls2k::port::ReferenceContinuityResult rejected_sparse_rows =
        ls2k::reference::BuildReferenceHoldCandidate(first_hold, changed_sparse_rows);
    Expect(!rejected_sparse_rows.hold_selected,
           "sparse row count identity change must reject hold output");
}

void TestReferencePathCompactsLeadingAndInteriorEmptyRows() {
    ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows(ls2k::port::kBevReferenceSampleCount);
    for (std::size_t index = 0; index < rows.size(); ++index) {
        rows[index].valid = true;
        rows[index].forward_m = params.bev_geometry.forward_samples_m[index];
    }

    auto add_interval = [&](std::size_t index, float center) {
        AddSyntheticInterval(rows[index], center - 0.08F, center + 0.08F);
    };

    add_interval(3, 0.0F);
    add_interval(4, 0.0F);
    add_interval(5, 0.0F);
    const ls2k::port::BEVReferencePath no_near =
        BuildSyntheticReferencePath(rows, params);
    Expect(ls2k::reference::EvaluateReferenceUsability(no_near, params).usable,
           "near missing rows must not make the first real continuous segment unusable");
    Expect(CountPresentPathPoints(no_near, ls2k::port::BEVPathPointSource::kIntervalCenter) == 3,
           "reference builder must publish the first real continuous segment");
    ExpectNear(no_near.sampled_path[0].point.forward_m,
               params.bev_geometry.forward_samples_m[3],
               1.0e-6F,
               "compact output must preserve the real forward distance of the first segment point");

    for (ls2k::vision::BEVSimpleRowScan& row : rows) {
        row.spans.clear();
        row.jumps.clear();
    }
    add_interval(0, 0.0F);
    add_interval(1, 0.0F);
    add_interval(2, 0.0F);
    add_interval(5, 0.0F);
    const ls2k::port::BEVReferencePath stopped =
        BuildSyntheticReferencePath(rows, params);
    Expect(ls2k::reference::EvaluateReferenceUsability(stopped, params).usable,
           "first three leading intervals satisfy the configured control minimum");
    Expect(CountPresentPathPoints(stopped, ls2k::port::BEVPathPointSource::kIntervalCenter) == 4,
           "reference builder must continue after empty rows and publish later connected candidates");
    ExpectNear(stopped.sampled_path[3].point.forward_m,
               params.bev_geometry.forward_samples_m[5],
               1.0e-6F,
               "compacted output after an interior gap must preserve true forward");
}

void TestOrdinaryReferenceInterpretsLostBoundaries() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.nominal_road_half_width_m = 0.21F;
    params.bev_geometry.lateral_step_m = 0.02F;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    rows.reserve(ls2k::port::kBevReferenceSampleCount);
    for (float forward : params.bev_geometry.forward_samples_m) {
        rows.push_back(SyntheticRow(forward, -1.0F, 1.0F));
    }

    for (std::size_t index = 0; index < 3U; ++index) {
        AddSyntheticInterval(rows[index], -0.20F, 0.20F);
    }
    const ls2k::port::BEVReferencePath both_edges =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(both_edges,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 3,
           "both-edge rows must produce leading midpoint reference samples");
    ExpectNear(both_edges.sampled_path[0].point.lateral_m, 0.0F, 1.0e-5F,
               "both-edge midpoint must remain the ordinary center");

    for (ls2k::vision::BEVSimpleRowScan& row : rows) {
        row.spans.clear();
        row.jumps.clear();
    }
    for (std::size_t index = 0; index < 4U; ++index) {
        AddSyntheticInterval(rows[index], -0.21F, 1.0F);
    }
    const ls2k::port::BEVReferencePath low_edge =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(low_edge,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 3,
           "low-edge-only rows must produce ordinary visual samples");
    ExpectNear(low_edge.sampled_path[0].point.lateral_m, 0.0F, 1.0e-5F,
               "low-edge-only rows must offset by positive nominal half width");

    for (ls2k::vision::BEVSimpleRowScan& row : rows) {
        row.spans.clear();
        row.jumps.clear();
    }
    for (std::size_t index = 0; index < 4U; ++index) {
        AddSyntheticInterval(rows[index], -1.0F, 0.21F);
    }
    const ls2k::port::BEVReferencePath high_edge =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(high_edge,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 3,
           "high-edge-only rows must produce ordinary visual samples");
    ExpectNear(high_edge.sampled_path[0].point.lateral_m, 0.0F, 1.0e-5F,
               "high-edge-only rows must offset by negative nominal half width");

    for (ls2k::vision::BEVSimpleRowScan& row : rows) {
        row.spans.clear();
        row.jumps.clear();
    }
    AddSyntheticInterval(rows[0], -1.0F, 1.0F);
    AddSyntheticInterval(rows[1], -0.20F, 0.20F);
    AddSyntheticInterval(rows[2], -0.20F, 0.20F);
    const ls2k::port::BEVReferencePath double_lost =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(double_lost,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 2,
           "double-lost leading row must not discard the later real segment");
    Expect(!ls2k::reference::EvaluateReferenceUsability(double_lost, params).usable,
           "two real points after a leading lost row must remain unusable");
}

void TestSparseRowMidpointUsesSharedConnectivityHelper() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 2.0F;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    rows.reserve(3U);
    for (float forward : std::array<float, 3U>{1.0F, 2.0F, 3.0F}) {
        rows.push_back(SyntheticRow(forward, 0.0F, 9.0F));
        AddSyntheticInterval(rows.back(), 1.0F, 8.0F);
    }

    const ls2k::port::BEVReferencePath reference =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(reference,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 3,
           "boundary span row facts must remain eligible midpoint candidates");
}

void TestBoundaryContinuityRejectsDiscontinuousSingleEdge() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.nominal_road_half_width_m = 0.21F;
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.45F;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    rows.reserve(ls2k::port::kBevReferenceSampleCount);
    for (float forward : params.bev_geometry.forward_samples_m) {
        rows.push_back(SyntheticRow(forward, -1.0F, 1.0F));
    }

    AddSyntheticInterval(rows[0], -0.21F, 1.0F);
    AddSyntheticInterval(rows[1], 0.80F, 1.0F);
    AddSyntheticInterval(rows[2], 0.80F, 1.0F);

    const ls2k::port::BEVReferencePath reference =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(reference,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 1,
           "discontinuous first single-edge row must not bridge into the later trace");
}

void TestBoundaryContinuityUsesFartherSupportAfterOutlier() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.nominal_road_half_width_m = 0.21F;
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.45F;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    rows.reserve(ls2k::port::kBevReferenceSampleCount);
    for (float forward : params.bev_geometry.forward_samples_m) {
        rows.push_back(SyntheticRow(forward, -1.0F, 1.0F));
    }

    AddSyntheticInterval(rows[0], -0.21F, 1.0F);
    AddSyntheticInterval(rows[1], 0.80F, 1.0F);
    AddSyntheticInterval(rows[2], -0.22F, 1.0F);

    const ls2k::port::BEVReferencePath reference =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(reference,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 0,
           "single-edge trace must not skip an outlier row to use farther support");
}

void TestBoundaryContinuityRequiresFutureSupportForSingleEdge() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.nominal_road_half_width_m = 0.21F;
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.45F;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    rows.reserve(ls2k::port::kBevReferenceSampleCount);
    for (float forward : params.bev_geometry.forward_samples_m) {
        rows.push_back(SyntheticRow(forward, -1.0F, 1.0F));
    }

    AddSyntheticInterval(rows[0], -0.21F, 1.0F);
    AddSyntheticInterval(rows[1], -0.21F, 1.0F);
    AddSyntheticInterval(rows[2], 0.80F, 1.0F);

    const ls2k::port::BEVReferencePath reference =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(reference,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 1,
           "single-edge row must not use only a previous kept edge as offset support");
}

void TestBoundaryContinuityDegradesOneClippedSide() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.nominal_road_half_width_m = 0.21F;
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.45F;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    rows.reserve(ls2k::port::kBevReferenceSampleCount);
    for (float forward : params.bev_geometry.forward_samples_m) {
        rows.push_back(SyntheticRow(forward, -1.5F, 1.5F));
    }

    AddSyntheticInterval(rows[0], -0.21F, 0.21F);
    AddSyntheticInterval(rows[1], -0.21F, 0.90F);
    AddSyntheticInterval(rows[2], -0.21F, 0.90F);

    const ls2k::port::BEVReferencePath reference =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(reference,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 3,
           "one clipped side must degrade to existing single-edge semantics");
    ExpectNear(reference.sampled_path[1].point.lateral_m,
               0.0F,
               0.005F,
               "row with clipped high edge must use low-edge offset, not raw midpoint");
}

void TestBoundaryContinuityRemovesRowWhenBothSidesClip() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.45F;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    rows.reserve(ls2k::port::kBevReferenceSampleCount);
    for (float forward : params.bev_geometry.forward_samples_m) {
        rows.push_back(SyntheticRow(forward, -1.5F, 1.5F));
    }

    AddSyntheticInterval(rows[0], -0.20F, 0.20F);
    AddSyntheticInterval(rows[1], 0.60F, 1.00F);
    AddSyntheticInterval(rows[2], -0.20F, 0.20F);

    const ls2k::port::BEVReferencePath reference =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(reference,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 2,
           "row with both edge facts clipped must create no candidate while later rows remain eligible");
}

void TestSingleEdgeOffsetMayLeaveSampleableSpan() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.nominal_road_half_width_m = 0.21F;
    params.bev_geometry.lateral_step_m = 0.02F;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    rows.reserve(ls2k::port::kBevReferenceSampleCount);
    for (float forward : params.bev_geometry.forward_samples_m) {
        rows.push_back(SyntheticRow(forward, -1.0F, 1.0F));
    }

    for (std::size_t index = 0; index < 4U; ++index) {
        AddSyntheticInterval(rows[index], -1.0F, -0.90F);
    }
    const ls2k::port::BEVReferencePath reference =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(reference,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 3,
           "visible single-edge rows may offset the center outside the sampleable span");
    Expect(reference.sampled_path[0].point.lateral_m < rows[0].sampleable_left_m,
           "single-edge offset must not be clamped back to the screen/sampleable edge");
    ExpectNear(reference.sampled_path[0].point.lateral_m, -1.11F, 1.0e-5F,
               "single-edge offset must be based on the visible edge, not the clipped edge");
}

void TestOrdinaryReferenceSelectsAfterCandidateInterpretation() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.nominal_road_half_width_m = 0.21F;
    params.bev_geometry.lateral_step_m = 0.02F;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    rows.reserve(ls2k::port::kBevReferenceSampleCount);
    for (float forward : params.bev_geometry.forward_samples_m) {
        rows.push_back(SyntheticRow(forward, -1.0F, 1.0F));
    }

    for (std::size_t index = 0; index < 3U; ++index) {
        AddSyntheticInterval(rows[index], -0.21F, 1.0F);
        AddSyntheticInterval(rows[index], 0.60F, 0.80F);
    }
    AddSyntheticInterval(rows[3], -0.21F, 1.0F);
    ScriptedConnectivityQuery interpreted_center_query({
        ls2k::vision::BEVSegmentConnectivityStatus::kBlocked,
        ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
        ls2k::vision::BEVSegmentConnectivityStatus::kBlocked,
        ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
        ls2k::vision::BEVSegmentConnectivityStatus::kBlocked,
        ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
    });
    const ls2k::port::BEVReferencePath reference =
        ls2k::vision::BuildReferencePath(rows, params, interpreted_center_query);
    Expect(CountPresentPathPoints(reference,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 3,
           "multi-interval one-side-lost rows must still produce a connected trace");
    ExpectNear(reference.sampled_path[0].point.lateral_m, 0.0F, 1.0e-5F,
               "connectivity selection must be able to accept an interpreted center after rejecting a midpoint");

    const ls2k::port::ReferenceHoldState hold =
        ls2k::reference::MakeReferenceHoldState(reference, params);
    for (ls2k::vision::BEVSimpleRowScan& row : rows) {
        row.spans.clear();
        row.jumps.clear();
    }
    AddSyntheticInterval(rows[0], -1.0F, 1.0F);
    const ls2k::port::BEVReferencePath unavailable =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(unavailable,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 0,
           "unavailable current visual reference must remain empty before hold");
    const ls2k::port::ReferenceContinuityResult held =
        ls2k::reference::BuildReferenceHoldCandidate(hold, params);
    Expect(held.hold_selected,
           "existing hold bridge must remain responsible for double-lost continuity");
}

void TestBoundaryJumpConnectivityRejectsLateralCrossing() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.lateral_step_m = 0.02F;
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 1.0F;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    rows.reserve(ls2k::port::kBevReferenceSampleCount);
    for (float forward : params.bev_geometry.forward_samples_m) {
        rows.push_back(SyntheticRow(forward, -1.0F, 1.0F));
    }
    AddSyntheticInterval(rows[0], -0.50F, -0.30F);
    AddSyntheticInterval(rows[1], 0.30F, 0.50F);
    AddSyntheticInterval(rows[2], 0.30F, 0.50F);

    const ls2k::port::BEVReferencePath reference =
        BuildSyntheticReferencePath(rows, params);
    Expect(CountPresentPathPoints(reference,
                                  ls2k::port::BEVPathPointSource::kIntervalCenter) == 3,
           "synthetic candidate generation must not infer image-segment connectivity");
}

void TestProjectionLutMatchesUncachedSparseScanAndRebuildsOnIdentityChange() {
    ls2k::port::RuntimeParameters params{};
    ls2k::vision::BEVProjector projector = MakeProjector(params);
    ls2k::port::LegacyCameraFrame frame = MakeFrame(0U);
    DrawVehicleStripe(frame, projector, params, -0.12F, 0.16F, 255U);

    ls2k::vision::BEVSampleProjectionLut lut{};
    TestPixelFrame pixel = MakeYuyvPixelFrame(frame, 7U, 7U);
    const ls2k::vision::BEVSimplePerceptionResult cached =
        ls2k::vision::RunBEVSimplePerception(pixel.view, params, projector, &lut);
    const ls2k::vision::BEVSimplePerceptionResult uncached =
        ls2k::vision::RunBEVSimplePerception(pixel.view, params, projector, nullptr);

    Expect(lut.valid, "sparse projection LUT must be built for a valid frame/projector identity");
    Expect(lut.entries.size() ==
               ls2k::port::kBevReferenceSampleCount * lut.lateral_sample_count,
           "LUT entry count must be forward rows times lateral samples");
    bool saw_sampleable = false;
    bool saw_non_sampleable = false;
    for (const ls2k::vision::BEVSampleProjectionEntry& entry : lut.entries) {
        saw_sampleable =
            saw_sampleable || entry.state == ls2k::vision::BEVSampleProjectionState::kSampleable;
        saw_non_sampleable =
            saw_non_sampleable || entry.state != ls2k::vision::BEVSampleProjectionState::kSampleable;
    }
    Expect(saw_sampleable, "LUT must mark in-frame projected samples as sampleable");
    Expect(saw_non_sampleable, "LUT must preserve out-of-frame or failed projection state separately");
    for (std::size_t index = 0; index < cached.rows.size(); ++index) {
        Expect(cached.rows[index].spans.size() == uncached.rows[index].spans.size(),
               "cached and uncached sparse scans must expose the same boundary span count");
        Expect(cached.rows[index].jumps.size() == uncached.rows[index].jumps.size(),
               "cached and uncached sparse scans must expose the same boundary jump count");
        if (!cached.rows[index].spans.empty()) {
            const ls2k::vision::BEVBoundarySpan& lhs = cached.rows[index].spans.front();
            const ls2k::vision::BEVBoundarySpan& rhs = uncached.rows[index].spans.front();
            const float center_tol = 0.5F * params.bev_geometry.lateral_step_m + 1.0e-4F;
            const float width_tol = 1.0F * params.bev_geometry.lateral_step_m + 1.0e-4F;
            Expect(std::abs(lhs.center_m - rhs.center_m) <= center_tol,
                   "LUT and non-LUT boundary span centers must match within sparse half-step tolerance");
            Expect(std::abs(lhs.width_m - rhs.width_m) <= width_tol,
                   "LUT and non-LUT boundary span widths must match within sparse one-step tolerance");
        }
    }

    const std::uint64_t previous_entry_count = static_cast<std::uint64_t>(lut.entries.size());
    params.bev_geometry.lateral_step_m *= 0.5F;
    Expect(ls2k::vision::EnsureBEVSampleProjectionLut(
               lut,
               pixel.view,
               params,
               projector),
           "LUT must rebuild successfully after sampling identity changes");
    Expect(static_cast<std::uint64_t>(lut.entries.size()) != previous_entry_count,
           "lateral step identity change must rebuild LUT with a different entry count");
}

void TestSparseRowCountUsesOriginalForwardSamplePrefix() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.sparse_row_count = 12;
    ls2k::vision::BEVProjector projector = MakeProjector(params);
    ls2k::port::LegacyCameraFrame frame = MakeFrame(0U);
    DrawVehicleStripe(frame, projector, params, -0.18F, 0.18F, 255U);

    ls2k::vision::BEVSampleProjectionLut lut{};
    TestPixelFrame pixel = MakeYuyvPixelFrame(frame, 9U, 9U);
    const ls2k::vision::BEVSimplePerceptionResult result =
        ls2k::vision::RunBEVSimplePerception(pixel.view, params, projector, &lut);

    Expect(result.rows.size() == 12U,
           "SPARSE_ROW_COUNT=12 must scan exactly the first 12 sparse rows");
    Expect(lut.sparse_row_count == 12U,
           "projection LUT must record the active sparse row prefix length");
    Expect(lut.entries.size() == 12U * lut.lateral_sample_count,
           "projection LUT must allocate only active sparse rows");
    for (std::size_t index = 0; index < result.rows.size(); ++index) {
        ExpectNear(result.rows[index].forward_m,
                   params.bev_geometry.forward_samples_m[index],
                   1.0e-6F,
                   "active sparse rows must keep the original forward sample positions");
    }
    for (std::size_t index = 12U; index < result.reference_path.sampled_path.size(); ++index) {
        Expect(!result.reference_path.sampled_path[index].present,
               "disabled sparse rows must remain absent in the reference path");
    }
}

void TestOriginToLastActiveRowMidpointConnectivity() {
    ls2k::port::RuntimeParameters default_params{};
    ls2k::vision::BEVProjector default_projector = MakeProjector(default_params);
    ls2k::port::LegacyCameraFrame default_uniform_frame = MakeFrame(100U);
    TestPixelFrame default_uniform_pixel =
        MakeYuyvPixelFrame(default_uniform_frame, 9U, 9U);
    ls2k::vision::BEVSampleProjectionLut default_lut{};
    const ls2k::vision::BEVSimplePerceptionResult default_result =
        ls2k::vision::RunBEVSimplePerception(
            default_uniform_pixel.view, default_params, default_projector, &default_lut);
    Expect(default_result.origin_to_last_row_midpoint_connectivity.status ==
               ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
           "uniform center segment must connect through the default last active row");

    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.sparse_row_count = 12;
    ls2k::vision::BEVProjector projector = MakeProjector(params);
    ls2k::port::LegacyCameraFrame uniform_frame = MakeFrame(100U);
    TestPixelFrame uniform_pixel = MakeYuyvPixelFrame(uniform_frame, 10U, 10U);
    ls2k::vision::BEVSampleProjectionLut uniform_lut{};

    const ls2k::vision::BEVSimplePerceptionResult uniform_result =
        ls2k::vision::RunBEVSimplePerception(
            uniform_pixel.view, params, projector, &uniform_lut);
    const ls2k::vision::BEVImageSegmentConnectivity direct_query(
        uniform_pixel.view, projector, params.bev_boundary);
    const ls2k::vision::BEVSegmentConnectivityResult expected =
        direct_query.Evaluate(
            {0.0F, 0.0F},
            {params.bev_geometry.forward_samples_m[11], 0.0F},
            ls2k::vision::BEVSegmentVisibilityPolicy::kAllowFromEndpointClip);

    Expect(uniform_result.origin_to_last_row_midpoint_connectivity.status == expected.status,
           "BEV perception must reuse the direct origin-to-last-active-row connectivity result");
    Expect(uniform_result.origin_to_last_row_midpoint_connectivity.sampled_point_count ==
               expected.sampled_point_count,
           "origin-to-last-active-row query must use the configured sparse prefix endpoint");
    Expect(uniform_result.origin_to_last_row_midpoint_connectivity.visible_segment_clipped ==
               expected.visible_segment_clipped,
           "origin-to-last-active-row query must preserve the original visibility policy");
    Expect(expected.status == ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
           "uniform center segment must be connected");

    ls2k::port::ImagePoint from_image{};
    ls2k::port::ImagePoint to_image{};
    Expect(projector.ProjectVehicleToImage({0.0F, 0.0F}, from_image) &&
               projector.ProjectVehicleToImage(
                   {params.bev_geometry.forward_samples_m[11], 0.0F}, to_image),
           "origin and last active row midpoint must project for the integration fixture");
    ls2k::port::LegacyCameraFrame blocked_frame = uniform_frame;
    DrawPatch(blocked_frame,
              from_image.row_px + 0.75F * (to_image.row_px - from_image.row_px),
              from_image.col_px + 0.75F * (to_image.col_px - from_image.col_px),
              255U);
    TestPixelFrame blocked_pixel = MakeYuyvPixelFrame(blocked_frame, 11U, 11U);
    ls2k::vision::BEVSampleProjectionLut blocked_lut{};
    const ls2k::vision::BEVSimplePerceptionResult blocked_result =
        ls2k::vision::RunBEVSimplePerception(
            blocked_pixel.view, params, projector, &blocked_lut);
    Expect(blocked_result.origin_to_last_row_midpoint_connectivity.status ==
               ls2k::vision::BEVSegmentConnectivityStatus::kBlocked,
           "a center-segment luma jump must block origin-to-last-row connectivity");

    const ls2k::port::CameraPixelFrameView invalid_frame{};
    ls2k::vision::BEVSampleProjectionLut invalid_lut{};
    const ls2k::vision::BEVSimplePerceptionResult invalid_result =
        ls2k::vision::RunBEVSimplePerception(
            invalid_frame, params, projector, &invalid_lut);
    Expect(invalid_result.rows.empty(), "invalid frame must not produce sparse rows");
    Expect(invalid_result.origin_to_last_row_midpoint_connectivity.status ==
               ls2k::vision::BEVSegmentConnectivityStatus::kUnobservable,
           "missing sparse rows must keep center-segment connectivity unobservable");
}

void TestRoadPathFactsStayAlignedWithSelectedMidpointCandidate() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 2.0F;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    for (std::size_t index = 0U; index < 3U; ++index) {
        rows.push_back(SyntheticRow(
            params.bev_geometry.forward_samples_m[index], -1.0F, 1.0F));
        AddSyntheticInterval(rows.back(), -0.60F, -0.40F);
        AddSyntheticInterval(rows.back(), 0.30F, 0.50F);
    }
    ScriptedConnectivityQuery query({
        ls2k::vision::BEVSegmentConnectivityStatus::kBlocked,
        ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
    });
    const ls2k::vision::BEVRoadPathFacts facts =
        ls2k::vision::BuildConnectedRoadPathFacts(rows, params, query);

    Expect(facts.center[0].present &&
               facts.actual_left_boundary[0].present &&
               facts.actual_right_boundary[0].present,
           "selected midpoint candidate must carry its two actual observed boundaries");
    ExpectNear(facts.center[0].point.lateral_m, 0.40F, 1.0e-6F,
               "selected midpoint center must remain aligned at output index zero");
    ExpectNear(facts.actual_left_boundary[0].point.lateral_m, 0.30F, 1.0e-6F,
               "selected midpoint must carry the left edge from its own span");
    ExpectNear(facts.actual_right_boundary[0].point.lateral_m, 0.50F, 1.0e-6F,
               "selected midpoint must carry the right edge from its own span");
}

void TestSingleEdgeFactsDoNotSynthesizeOppositeBoundary() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.nominal_road_half_width_m = 0.21F;
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.45F;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows;
    for (std::size_t index = 0U; index < 4U; ++index) {
        rows.push_back(SyntheticRow(
            params.bev_geometry.forward_samples_m[index], -1.0F, 1.0F));
        AddSyntheticInterval(rows.back(), -0.21F, 1.0F);
    }
    const AlwaysConnectedQuery query{};
    const ls2k::vision::BEVRoadPathFacts facts =
        ls2k::vision::BuildConnectedRoadPathFacts(rows, params, query);

    for (std::size_t index = 0U; index < 3U; ++index) {
        Expect(facts.center[index].present,
               "supported single edge must still provide an ordinary center fact");
        Expect(facts.actual_left_boundary[index].present,
               "single low edge must be retained as an actual left boundary");
        Expect(!facts.actual_right_boundary[index].present,
               "single low edge must not synthesize an opposite right boundary");
        ExpectNear(facts.actual_left_boundary[index].point.lateral_m, -0.21F, 1.0e-6F,
                   "single-edge fact must retain the literal observed lateral coordinate");
    }

    const ls2k::port::BEVReferencePath literal_left =
        ls2k::vision::BuildObservedBoundaryReferencePath(
            facts, ls2k::vision::ObservedBoundarySide::kLeft, 3U);
    Expect(literal_left.mode == ls2k::port::ReferenceMode::kMlObservedBoundary,
           "three literal boundary samples must produce ML observed-boundary mode");
    Expect(CountPresentPathPoints(
               literal_left,
               ls2k::port::BEVPathPointSource::kMlObservedBoundary) == 3,
           "literal boundary output must label every sample with ML observed-boundary source");
    ExpectNear(literal_left.sampled_path[0].point.lateral_m, -0.21F, 1.0e-6F,
               "ML observed-boundary output must not apply the ordinary center offset");

    const ls2k::port::BEVReferencePath missing_right =
        ls2k::vision::BuildObservedBoundaryReferencePath(
            facts, ls2k::vision::ObservedBoundarySide::kRight, 3U);
    Expect(missing_right.mode == ls2k::port::ReferenceMode::kNone &&
               !missing_right.sampled_path[0].present,
           "missing opposite boundary must remain unavailable to the ML path builder");

    const ls2k::port::BEVReferencePath insufficient_left =
        ls2k::vision::BuildObservedBoundaryReferencePath(
            facts, ls2k::vision::ObservedBoundarySide::kLeft, 4U);
    Expect(insufficient_left.mode == ls2k::port::ReferenceMode::kNone &&
               !insufficient_left.sampled_path[0].present,
           "fewer than caller-required boundary samples must produce no path");
}

}  // namespace

int main() {
    try {
        TestSingleBoundaryOffsetHelperGeometry();
        TestBoundaryTraceClipHelperRule();
        TestBevLocalBoundaryFacts();
        TestBevGeometryControlsWideImageScan();
        TestHoldIsExplicitNonVisualSource();
        TestConnectedPathSkipsFailuresAndPreservesPredecessor();
        TestConnectedPathAcceptsFirstPassingCandidateInProductionOrder();
        TestReferencePathCompactsLeadingAndInteriorEmptyRows();
        TestOrdinaryReferenceInterpretsLostBoundaries();
        TestSparseRowMidpointUsesSharedConnectivityHelper();
        TestBoundaryContinuityRejectsDiscontinuousSingleEdge();
        TestBoundaryContinuityUsesFartherSupportAfterOutlier();
        TestBoundaryContinuityRequiresFutureSupportForSingleEdge();
        TestBoundaryContinuityDegradesOneClippedSide();
        TestBoundaryContinuityRemovesRowWhenBothSidesClip();
        TestSingleEdgeOffsetMayLeaveSampleableSpan();
        TestOrdinaryReferenceSelectsAfterCandidateInterpretation();
        TestBoundaryJumpConnectivityRejectsLateralCrossing();
        TestProjectionLutMatchesUncachedSparseScanAndRebuildsOnIdentityChange();
        TestSparseRowCountUsesOriginalForwardSamplePrefix();
        TestOriginToLastActiveRowMidpointConnectivity();
        TestRoadPathFactsStayAlignedWithSelectedMidpointCandidate();
        TestSingleEdgeFactsDoNotSynthesizeOppositeBoundary();
    } catch (const TestFailure& failure) {
        std::cerr << "bev_simple_perception_test failed: " << failure.message << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "bev_simple_perception_test passed\n";
    return EXIT_SUCCESS;
}
