#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "vision/elements/cross_straight_path_planner.hpp"

namespace {

struct TestFailure {
    std::string message;
};

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw TestFailure{message};
    }
}

class Connectivity final : public ls2k::vision::BEVSegmentConnectivityQuery {
public:
    explicit Connectivity(ls2k::vision::BEVSegmentConnectivityStatus status)
        : status_(status) {}

    ls2k::vision::BEVSegmentConnectivityResult Evaluate(
        const ls2k::port::BEVPoint&,
        const ls2k::port::BEVPoint&,
        ls2k::vision::BEVSegmentVisibilityPolicy) const override {
        return {status_, 8U, false};
    }

private:
    ls2k::vision::BEVSegmentConnectivityStatus status_;
};

class RejectOneDestinationConnectivity final
    : public ls2k::vision::BEVSegmentConnectivityQuery {
public:
    explicit RejectOneDestinationConnectivity(float rejected_forward_m)
        : rejected_forward_m_(rejected_forward_m) {}

    ls2k::vision::BEVSegmentConnectivityResult Evaluate(
        const ls2k::port::BEVPoint&,
        const ls2k::port::BEVPoint& to,
        ls2k::vision::BEVSegmentVisibilityPolicy) const override {
        const bool rejected = std::abs(to.forward_m - rejected_forward_m_) < 1.0e-5F;
        return {rejected ? ls2k::vision::BEVSegmentConnectivityStatus::kBlocked
                         : ls2k::vision::BEVSegmentConnectivityStatus::kConnected,
                8U,
                false};
    }

private:
    float rejected_forward_m_ = 0.0F;
};

ls2k::vision::BEVWhiteRun Run(
    float left_m,
    float right_m,
    ls2k::vision::BEVWhiteRunEndpointState left_endpoint,
    ls2k::vision::BEVWhiteRunEndpointState right_endpoint,
    ls2k::vision::BEVWhiteRunOriginConnectivity origin) {
    ls2k::vision::BEVWhiteRun run{};
    run.left_m = left_m;
    run.right_m = right_m;
    run.left_endpoint = left_endpoint;
    run.right_endpoint = right_endpoint;
    run.origin_connectivity = origin;
    return run;
}

ls2k::vision::BEVSimpleRowScan Row(float forward_m,
                                   const ls2k::vision::BEVWhiteRun& route,
                                   bool add_wrong_branch = true) {
    ls2k::vision::BEVSimpleRowScan row{};
    row.valid = true;
    row.forward_m = forward_m;
    row.sampleable_count = 80U;
    row.white_runs.push_back(route);
    if (add_wrong_branch) {
        row.white_runs.push_back(Run(-0.15F,
                                     0.35F,
                                     ls2k::vision::BEVWhiteRunEndpointState::kBoundary,
                                     ls2k::vision::BEVWhiteRunEndpointState::kBoundary,
                                     ls2k::vision::BEVWhiteRunOriginConnectivity::kBlocked));
    }
    return row;
}

std::vector<ls2k::vision::BEVSimpleRowScan> StraightThroughRows() {
    using Endpoint = ls2k::vision::BEVWhiteRunEndpointState;
    using Origin = ls2k::vision::BEVWhiteRunOriginConnectivity;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
    rows.push_back(Row(0.10F, Run(-0.70F, 0.00F, Endpoint::kFovEdge, Endpoint::kBoundary,
                                 Origin::kConnected)));
    rows.push_back(Row(0.20F, Run(-0.78F, -0.08F, Endpoint::kFovEdge, Endpoint::kBoundary,
                                 Origin::kConnected)));
    rows.push_back(Row(0.30F, Run(-0.86F, -0.16F, Endpoint::kFovEdge, Endpoint::kBoundary,
                                 Origin::kConnected)));
    rows.push_back(Row(0.40F, Run(-0.94F, -0.24F, Endpoint::kFovEdge, Endpoint::kBoundary,
                                 Origin::kUnknown)));
    rows.push_back(Row(0.50F, Run(-1.02F, -0.32F, Endpoint::kFovEdge, Endpoint::kBoundary,
                                 Origin::kUnknown)));
    rows.push_back(Row(0.60F, Run(-1.10F, -0.55F, Endpoint::kBoundary, Endpoint::kBoundary,
                                 Origin::kUnknown)));
    rows.push_back(Row(0.70F, Run(-1.18F, -0.63F, Endpoint::kBoundary, Endpoint::kBoundary,
                                 Origin::kUnknown)));
    rows.push_back(Row(0.80F, Run(-1.26F, -0.71F, Endpoint::kBoundary, Endpoint::kBoundary,
                                 Origin::kUnknown)));
    return rows;
}

ls2k::port::CrossExitElementEvidence PresentCross() {
    ls2k::port::CrossExitElementEvidence evidence{};
    evidence.present = true;
    evidence.reason = "present";
    return evidence;
}

void TestFollowsConnectedRouteInsteadOfFirstSpan() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.25F;
    params.bev_geometry.nominal_road_half_width_m = 0.225F;
    const Connectivity connected(ls2k::vision::BEVSegmentConnectivityStatus::kConnected);
    const auto candidate = ls2k::vision::BuildCrossStraightPathCandidate(
        StraightThroughRows(), PresentCross(), params, connected);

    Expect(candidate.present, "a unique connected entry and bounded exit must produce a path");
    Expect(candidate.kind == ls2k::port::VisualReferenceCandidateKind::kCrossExit,
           "the planner must publish a Cross candidate");
    std::size_t count = 0U;
    float previous_lateral = 0.0F;
    for (const auto& sample : candidate.reference_path.sampled_path) {
        if (!sample.present) {
            break;
        }
        if (count > 0U) {
            Expect(sample.point.lateral_m < previous_lateral + 0.02F,
                   "the route must not jump toward the blocked middle branch");
        }
        previous_lateral = sample.point.lateral_m;
        ++count;
    }
    Expect(count == 8U, "the full observed straight-through route must be published");
    Expect(previous_lateral < -0.8F, "the far path must terminate in the selected left exit");
}

void TestAmbiguousOriginRowOnlyRemovesThatRow() {
    auto rows = StraightThroughRows();
    rows.front().white_runs.push_back(rows.front().white_runs.front());
    rows.front().white_runs.back().origin_connectivity =
        ls2k::vision::BEVWhiteRunOriginConnectivity::kConnected;
    const Connectivity connected(ls2k::vision::BEVSegmentConnectivityStatus::kConnected);
    const auto candidate = ls2k::vision::BuildCrossStraightPathCandidate(
        rows, PresentCross(), ls2k::port::RuntimeParameters{}, connected);
    Expect(candidate.present,
           "one ambiguous origin row must not reject later unambiguous route facts");
    Expect(std::abs(candidate.reference_path.sampled_path[0].point.forward_m - 0.20F) <
               1.0e-5F,
           "the ambiguous row alone must be omitted from the route");
}

void TestOneDisconnectedPortalOnlyRemovesThatPoint() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.25F;
    params.bev_geometry.nominal_road_half_width_m = 0.225F;
    const RejectOneDestinationConnectivity connectivity(0.40F);
    const auto candidate = ls2k::vision::BuildCrossStraightPathCandidate(
        StraightThroughRows(), PresentCross(), params, connectivity);

    Expect(candidate.present,
           "one disconnected portal must not reject the remaining connected path");
    std::size_t count = 0U;
    for (const auto& sample : candidate.reference_path.sampled_path) {
        if (!sample.present) {
            continue;
        }
        Expect(std::abs(sample.point.forward_m - 0.40F) > 1.0e-5F,
               "the disconnected portal alone must be omitted");
        ++count;
    }
    Expect(count == 7U, "all seven other connected Cross points must remain");
}

void TestOneEntryOutlierCannotRotateTheStraightAxis() {
    auto rows = StraightThroughRows();
    rows[3U].white_runs.front().right_m = 0.20F;

    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.50F;
    params.bev_geometry.nominal_road_half_width_m = 0.225F;
    const Connectivity connected(ls2k::vision::BEVSegmentConnectivityStatus::kConnected);
    const auto candidate = ls2k::vision::BuildCrossStraightPathCandidate(
        rows, PresentCross(), params, connected);

    Expect(candidate.present,
           "one displaced entry observation must not erase the straight route");
    Expect(candidate.reference_path.sampled_path[7U].point.lateral_m < -0.80F,
           "one displaced entry observation must not rotate selection into the side branch");
}

void TestOneMissingExitObservationDoesNotSplitTheExitLine() {
    using Endpoint = ls2k::vision::BEVWhiteRunEndpointState;
    auto rows = StraightThroughRows();
    rows[6U].white_runs.front().left_endpoint = Endpoint::kUnavailableGap;
    rows[6U].white_runs.front().right_endpoint = Endpoint::kUnavailableGap;

    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.30F;
    params.bev_geometry.nominal_road_half_width_m = 0.225F;
    const Connectivity connected(ls2k::vision::BEVSegmentConnectivityStatus::kConnected);
    const auto candidate = ls2k::vision::BuildCrossStraightPathCandidate(
        rows, PresentCross(), params, connected);

    Expect(candidate.present,
           "one row without an exit-center observation must not split the exit line");
    Expect(candidate.reference_path.sampled_path[7U].present,
           "the valid exit observation after the missing row must remain usable");
}

void TestStraightAxisResolvesAnOverlappingSplit() {
    using Endpoint = ls2k::vision::BEVWhiteRunEndpointState;
    using Origin = ls2k::vision::BEVWhiteRunOriginConnectivity;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
    rows.push_back(Row(0.10F,
                       Run(-0.70F, 0.00F, Endpoint::kFovEdge, Endpoint::kBoundary,
                           Origin::kConnected),
                       false));
    rows.push_back(Row(0.20F,
                       Run(-0.80F, -0.10F, Endpoint::kFovEdge, Endpoint::kBoundary,
                           Origin::kConnected),
                       false));
    rows.push_back(Row(0.30F,
                       Run(-0.90F, -0.20F, Endpoint::kFovEdge, Endpoint::kBoundary,
                           Origin::kConnected),
                       false));
    auto split = Row(0.40F,
                     Run(-0.20F, 0.50F, Endpoint::kBoundary, Endpoint::kBoundary,
                         Origin::kConnected),
                     false);
    split.white_runs.push_back(
        Run(-1.40F, -0.20F, Endpoint::kBoundary, Endpoint::kBoundary, Origin::kConnected));
    rows.push_back(split);
    rows.push_back(Row(0.50F,
                       Run(-1.50F, -0.70F, Endpoint::kBoundary, Endpoint::kBoundary,
                           Origin::kUnknown),
                       false));
    rows.push_back(Row(0.60F,
                       Run(-1.60F, -0.80F, Endpoint::kBoundary, Endpoint::kBoundary,
                           Origin::kUnknown),
                       false));

    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.25F;
    const Connectivity connected(ls2k::vision::BEVSegmentConnectivityStatus::kConnected);
    const auto candidate = ls2k::vision::BuildCrossStraightPathCandidate(
        rows, PresentCross(), params, connected);
    Expect(candidate.present,
           "the established route must resolve a split even when both runs connect to origin: " +
               candidate.reason);
    Expect(candidate.reference_path.sampled_path[3U].point.lateral_m < -0.2F,
           "candidate container order must not turn the Cross route toward the right branch");
}

void TestBlockedComposedPathIsRejected() {
    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.25F;
    const Connectivity blocked(ls2k::vision::BEVSegmentConnectivityStatus::kBlocked);
    const auto candidate = ls2k::vision::BuildCrossStraightPathCandidate(
        StraightThroughRows(), PresentCross(), params, blocked);
    Expect(!candidate.present, "a composed path crossing black pixels must be rejected");
    Expect(candidate.reason == "cross_route_corridor_disconnected",
           "connectivity rejection must remain observable at route composition");
}

void TestTransitionFollowsRoadCenterGuideInsteadOfTheShortestBoundary() {
    using Endpoint = ls2k::vision::BEVWhiteRunEndpointState;
    using Origin = ls2k::vision::BEVWhiteRunOriginConnectivity;
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
    rows.push_back(Row(0.10F,
                       Run(-0.60F, 0.00F, Endpoint::kFovEdge, Endpoint::kBoundary,
                           Origin::kConnected),
                       false));
    rows.push_back(Row(0.20F,
                       Run(-0.40F, 0.20F, Endpoint::kFovEdge, Endpoint::kBoundary,
                           Origin::kConnected),
                       false));
    rows.push_back(Row(0.30F,
                       Run(0.00F, 1.00F, Endpoint::kUnavailableGap,
                           Endpoint::kUnavailableGap, Origin::kUnknown),
                       false));
    rows.push_back(Row(0.40F,
                       Run(0.00F, 1.00F, Endpoint::kUnavailableGap,
                           Endpoint::kUnavailableGap, Origin::kUnknown),
                       false));
    rows.push_back(Row(0.50F,
                       Run(-0.20F, 0.20F, Endpoint::kBoundary, Endpoint::kBoundary,
                           Origin::kUnknown),
                       false));
    rows.push_back(Row(0.60F,
                       Run(-0.40F, 0.00F, Endpoint::kBoundary, Endpoint::kBoundary,
                           Origin::kUnknown),
                       false));

    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.25F;
    params.bev_geometry.nominal_road_half_width_m = 0.20F;
    const Connectivity connected(ls2k::vision::BEVSegmentConnectivityStatus::kConnected);
    const auto candidate = ls2k::vision::BuildCrossStraightPathCandidate(
        rows, PresentCross(), params, connected);

    Expect(candidate.present, "a connected entry-guide-exit route must be planned");
    const float transition_lateral_m =
        candidate.reference_path.sampled_path[2U].point.lateral_m;
    Expect(transition_lateral_m > 0.10F && transition_lateral_m < 0.20F,
           "the transition must follow the slope-continuous road-center guide, not the "
           "geometrically shortest white-run boundary");
}

void TestOneSidedFovRunCannotReplaceTheBoundedStraightExit() {
    using Endpoint = ls2k::vision::BEVWhiteRunEndpointState;
    using Origin = ls2k::vision::BEVWhiteRunOriginConnectivity;
    auto rows = StraightThroughRows();
    rows.push_back(Row(0.90F,
                       Run(-0.825F, 0.50F, Endpoint::kBoundary, Endpoint::kFovEdge,
                           Origin::kUnknown),
                       false));
    rows.push_back(Row(1.00F,
                       Run(-0.725F, 0.60F, Endpoint::kBoundary, Endpoint::kFovEdge,
                           Origin::kUnknown),
                       false));

    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.25F;
    params.bev_geometry.nominal_road_half_width_m = 0.225F;
    const Connectivity connected(ls2k::vision::BEVSegmentConnectivityStatus::kConnected);
    const auto candidate = ls2k::vision::BuildCrossStraightPathCandidate(
        rows, PresentCross(), params, connected);

    Expect(candidate.present, "the observed bounded straight exit must remain usable");
    std::size_t count = 0U;
    for (const auto& sample : candidate.reference_path.sampled_path) {
        if (!sample.present) {
            break;
        }
        ++count;
    }
    Expect(count == 8U,
           "a later one-sided FOV run must not replace or extend the bounded exit");
    Expect(candidate.reference_path.sampled_path[7U].point.lateral_m < -0.90F,
           "the route must terminate on the observed bounded exit center line");
}

void TestShortBoundedFragmentCannotReplaceTheSupportedStraightExit() {
    using Endpoint = ls2k::vision::BEVWhiteRunEndpointState;
    using Origin = ls2k::vision::BEVWhiteRunOriginConnectivity;
    auto rows = StraightThroughRows();
    rows.push_back(Row(0.90F,
                       Run(-0.80F, 0.20F, Endpoint::kBoundary, Endpoint::kBoundary,
                           Origin::kUnknown),
                       false));
    rows.push_back(Row(1.00F,
                       Run(-0.70F, 0.30F, Endpoint::kBoundary, Endpoint::kBoundary,
                           Origin::kUnknown),
                       false));

    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.25F;
    params.bev_geometry.nominal_road_half_width_m = 0.225F;
    const Connectivity connected(ls2k::vision::BEVSegmentConnectivityStatus::kConnected);
    const auto candidate = ls2k::vision::BuildCrossStraightPathCandidate(
        rows, PresentCross(), params, connected);

    Expect(candidate.present, "the meter-supported bounded exit must remain usable");
    Expect(!candidate.reference_path.sampled_path[8U].present,
           "a shorter later bounded fragment must not replace the supported exit");
}

void TestTrailingDisconnectedFactsDoNotInvalidateACompleteRoute() {
    using Endpoint = ls2k::vision::BEVWhiteRunEndpointState;
    using Origin = ls2k::vision::BEVWhiteRunOriginConnectivity;
    auto rows = StraightThroughRows();
    rows.push_back(Row(0.90F,
                       Run(0.40F, 0.80F, Endpoint::kBoundary, Endpoint::kBoundary,
                           Origin::kBlocked),
                       false));

    ls2k::port::RuntimeParameters params{};
    params.bev_geometry.boundary_trace_max_adjacent_distance_m = 0.25F;
    params.bev_geometry.nominal_road_half_width_m = 0.225F;
    const Connectivity connected(ls2k::vision::BEVSegmentConnectivityStatus::kConnected);
    const auto candidate = ls2k::vision::BuildCrossStraightPathCandidate(
        rows, PresentCross(), params, connected);

    Expect(candidate.present,
           "facts beyond the maximal origin-connected route must not erase a complete path");
    Expect(!candidate.reference_path.sampled_path[8U].present,
           "the path must end at the last continuous route row");
}

}  // namespace

int main() {
    try {
        TestFollowsConnectedRouteInsteadOfFirstSpan();
        TestAmbiguousOriginRowOnlyRemovesThatRow();
        TestOneDisconnectedPortalOnlyRemovesThatPoint();
        TestOneEntryOutlierCannotRotateTheStraightAxis();
        TestOneMissingExitObservationDoesNotSplitTheExitLine();
        TestStraightAxisResolvesAnOverlappingSplit();
        TestBlockedComposedPathIsRejected();
        TestTransitionFollowsRoadCenterGuideInsteadOfTheShortestBoundary();
        TestOneSidedFovRunCannotReplaceTheBoundedStraightExit();
        TestShortBoundedFragmentCannotReplaceTheSupportedStraightExit();
        TestTrailingDisconnectedFactsDoNotInvalidateACompleteRoute();
    } catch (const TestFailure& failure) {
        std::cerr << "cross_straight_path_planner_test failed: " << failure.message << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "cross_straight_path_planner_test passed\n";
    return EXIT_SUCCESS;
}
