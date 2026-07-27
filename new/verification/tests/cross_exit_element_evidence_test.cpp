#include <cstdlib>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "vision/elements/cross_exit_element_evidence.hpp"

namespace {

struct TestFailure {
    std::string message;
};

void Expect(bool condition, const std::string& message) {
    if (!condition) {
        throw TestFailure{message};
    }
}

ls2k::vision::BEVSimpleRowScan BoundaryAbsentRow(float forward_m) {
    ls2k::vision::BEVSimpleRowScan row{};
    row.valid = true;
    row.forward_m = forward_m;
    row.sampleable_count = 8U;
    row.sampleable_left_m = -0.4F;
    row.sampleable_right_m = 0.4F;
    ls2k::vision::BEVWhiteRun run{};
    run.left_endpoint = ls2k::vision::BEVWhiteRunEndpointState::kFovEdge;
    run.right_endpoint = ls2k::vision::BEVWhiteRunEndpointState::kFovEdge;
    run.origin_connectivity =
        ls2k::vision::BEVWhiteRunOriginConnectivity::kConnected;
    row.white_runs.push_back(run);
    return row;
}

ls2k::vision::BEVSimpleRowScan BoundedRoadRow(float forward_m) {
    auto row = BoundaryAbsentRow(forward_m);
    row.jumps.push_back({});
    row.white_runs.front().left_endpoint =
        ls2k::vision::BEVWhiteRunEndpointState::kBoundary;
    row.white_runs.front().right_endpoint =
        ls2k::vision::BEVWhiteRunEndpointState::kBoundary;
    return row;
}

std::vector<ls2k::vision::BEVSimpleRowScan> BoundaryAbsentRows(std::size_t count) {
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
    rows.push_back(BoundedRoadRow(0.1F));
    for (std::size_t index = 0; index < count; ++index) {
        rows.push_back(BoundaryAbsentRow(0.2F + 0.1F * static_cast<float>(index)));
    }
    return rows;
}

ls2k::vision::BEVSimpleRowScan ConnectedSingleBoundaryRow(float forward_m) {
    ls2k::vision::BEVSimpleRowScan row = BoundaryAbsentRow(forward_m);
    row.jumps.push_back({});
    row.white_runs.front().right_endpoint =
        ls2k::vision::BEVWhiteRunEndpointState::kBoundary;
    return row;
}

ls2k::vision::BEVSimpleRowScan ConnectedCrossOpenRow(float forward_m) {
    auto row = ConnectedSingleBoundaryRow(forward_m);
    row.white_runs.front().right_endpoint =
        ls2k::vision::BEVWhiteRunEndpointState::kFovEdge;
    return row;
}

ls2k::vision::BEVSimpleRowScan RightBoundaryWithLeftFov(float forward_m,
                                                        float right_m) {
    auto row = ConnectedSingleBoundaryRow(forward_m);
    row.white_runs.front().left_m = -0.4F;
    row.white_runs.front().right_m = right_m;
    return row;
}

ls2k::vision::BEVSimpleRowScan LeftBoundaryWithRightFov(float forward_m,
                                                        float left_m) {
    auto row = RightBoundaryWithLeftFov(forward_m, 0.4F);
    row.white_runs.front().left_endpoint =
        ls2k::vision::BEVWhiteRunEndpointState::kBoundary;
    row.white_runs.front().right_endpoint =
        ls2k::vision::BEVWhiteRunEndpointState::kFovEdge;
    row.white_runs.front().left_m = left_m;
    return row;
}

ls2k::vision::BEVSegmentConnectivityResult Connectivity(
    ls2k::vision::BEVSegmentConnectivityStatus status) {
    ls2k::vision::BEVSegmentConnectivityResult result{};
    result.status = status;
    result.sampled_point_count = 10U;
    return result;
}

void TestTwoRowsRemainAbsent() {
    const ls2k::port::RuntimeParameters params{};
    const ls2k::port::CrossExitElementEvidence evidence =
        ls2k::vision::DetectCrossExitEvidence(
            BoundaryAbsentRows(2U),
            Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
            params);
    Expect(!evidence.present, "two boundary-absent rows must remain below the cross threshold");
    Expect(evidence.boundary_absent_row_count == 2U,
           "two-row evidence must preserve the longest absence run");
    Expect(evidence.reason == "boundary_absence_rows_absent",
           "two-row evidence must preserve the existing absence reason");
}

void TestThreeRowsRemainPresent() {
    const ls2k::port::RuntimeParameters params{};
    const ls2k::port::CrossExitElementEvidence evidence =
        ls2k::vision::DetectCrossExitEvidence(
            BoundaryAbsentRows(3U),
            Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
            params);
    Expect(evidence.present, "three boundary-absent rows must still satisfy cross detection");
    Expect(evidence.boundary_absent_row_count == 3U,
           "three-row evidence must preserve the longest absence run");
    Expect(evidence.reason == "present_full_fov",
           "three-row evidence must expose the full-FOV source");
}

void TestRightBoundaryExpansionWithLeftFovIsCross() {
    const ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{
        RightBoundaryWithLeftFov(0.10F, 0.20F),
        RightBoundaryWithLeftFov(0.20F, 0.22F),
        RightBoundaryWithLeftFov(0.30F, 0.24F),
        RightBoundaryWithLeftFov(0.40F, 0.36F),
        RightBoundaryWithLeftFov(0.50F, 0.38F),
    };
    const auto evidence = ls2k::vision::DetectCrossExitEvidence(
        rows,
        Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
        params);
    Expect(evidence.present, "right boundary expansion with left FOV must define Cross");
    Expect(evidence.reason == "present_right_boundary_expansion",
           "right expansion must expose its source");
}

void TestLeftBoundaryExpansionWithRightFovIsCross() {
    const ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{
        LeftBoundaryWithRightFov(0.10F, -0.20F),
        LeftBoundaryWithRightFov(0.20F, -0.22F),
        LeftBoundaryWithRightFov(0.30F, -0.24F),
        LeftBoundaryWithRightFov(0.40F, -0.36F),
        LeftBoundaryWithRightFov(0.50F, -0.38F),
    };
    const auto evidence = ls2k::vision::DetectCrossExitEvidence(
        rows,
        Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
        params);
    Expect(evidence.present, "left boundary expansion with right FOV must define Cross");
    Expect(evidence.reason == "present_left_boundary_expansion",
           "left expansion must expose its source");
}

void TestExpansionUsesBoundaryRelativeDistanceUnderRotation() {
    const ls2k::port::RuntimeParameters params{};
    constexpr float kPi = 3.14159265358979323846F;
    for (const float angle_deg : {-15.0F, 15.0F}) {
        const float slope = std::tan(angle_deg * kPi / 180.0F);
        const float perpendicular_offset =
            0.10F * std::sqrt(1.0F + slope * slope);
        std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
        for (const float forward_m : {0.10F, 0.20F, 0.30F}) {
            rows.push_back(RightBoundaryWithLeftFov(
                forward_m, 0.20F + slope * forward_m));
        }
        for (const float forward_m : {0.40F, 0.50F}) {
            rows.push_back(RightBoundaryWithLeftFov(
                forward_m, 0.20F + slope * forward_m + perpendicular_offset));
        }
        const auto evidence = ls2k::vision::DetectCrossExitEvidence(
            rows,
            Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
            params);
        Expect(evidence.present,
               "rotating the whole boundary must not erase metric expansion");
    }
}

void TestSingleExpandedRowIsCurrentFrameCross() {
    const ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{
        RightBoundaryWithLeftFov(0.10F, 0.20F),
        RightBoundaryWithLeftFov(0.20F, 0.22F),
        RightBoundaryWithLeftFov(0.30F, 0.24F),
        RightBoundaryWithLeftFov(0.40F, 0.36F),
    };
    const auto evidence = ls2k::vision::DetectCrossExitEvidence(
        rows,
        Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
        params);
    Expect(evidence.present,
           "one expanded row with a valid baseline and opposite FOV must define Cross");
}

void TestExpansionRequiresOppositeFov() {
    const ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{
        RightBoundaryWithLeftFov(0.10F, 0.20F),
        RightBoundaryWithLeftFov(0.20F, 0.22F),
        RightBoundaryWithLeftFov(0.30F, 0.24F),
        BoundedRoadRow(0.40F),
        BoundedRoadRow(0.50F),
    };
    rows[3].white_runs.front().right_m = 0.36F;
    rows[4].white_runs.front().right_m = 0.38F;
    const auto evidence = ls2k::vision::DetectCrossExitEvidence(
        rows,
        Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
        params);
    Expect(!evidence.present, "boundary expansion without opposite FOV must remain ordinary");
}

void TestNoTransitionWithoutConnectedWhiteRunIsNotOpen() {
    const ls2k::port::RuntimeParameters params{};
    auto rows = BoundaryAbsentRows(3U);
    for (auto& row : rows) {
        row.jumps.clear();
        row.spans.clear();
        row.white_runs.clear();
    }
    const auto evidence = ls2k::vision::DetectCrossExitEvidence(
        rows,
        Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
        params);
    Expect(!evidence.present,
           "no jumps or spans without a connected white run must not imply a Cross opening");
}

void TestConnectedCrossOpeningIgnoresUnrelatedRowBoundary() {
    const ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
    rows.push_back(ConnectedSingleBoundaryRow(0.16F));
    rows.push_back(ConnectedCrossOpenRow(0.20F));
    rows.push_back(ConnectedCrossOpenRow(0.24F));
    rows.push_back(ConnectedCrossOpenRow(0.28F));
    const auto evidence = ls2k::vision::DetectCrossExitEvidence(
        rows,
        Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
        params);
    Expect(evidence.present,
           "the unique origin-connected run reaching both FOV edges must define a Cross row");
}

void TestBoundedRoadLosingOneSideRemainsOrdinary() {
    const ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
    rows.push_back(BoundedRoadRow(0.16F));
    rows.push_back(ConnectedSingleBoundaryRow(0.20F));
    rows.push_back(ConnectedSingleBoundaryRow(0.24F));
    rows.push_back(ConnectedSingleBoundaryRow(0.28F));
    const auto evidence = ls2k::vision::DetectCrossExitEvidence(
        rows,
        Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
        params);
    Expect(!evidence.present,
           "a bounded road losing one side must remain an ordinary single-boundary road");
}

void TestSingleBoundaryRunRemainsOrdinaryRoad() {
    const ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
    rows.push_back(ConnectedSingleBoundaryRow(0.20F));
    rows.push_back(ConnectedSingleBoundaryRow(0.24F));
    rows.push_back(ConnectedSingleBoundaryRow(0.28F));
    const auto evidence = ls2k::vision::DetectCrossExitEvidence(
        rows,
        Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
        params);
    Expect(!evidence.present,
           "one observed boundary plus one FOV edge must remain a normal single-boundary road");
}

void TestBoundaryBoundConnectedRunIsNotOpen() {
    const ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
    for (float forward_m : {0.20F, 0.24F, 0.28F}) {
        auto row = ConnectedSingleBoundaryRow(forward_m);
        row.white_runs.front().left_endpoint =
            ls2k::vision::BEVWhiteRunEndpointState::kBoundary;
        rows.push_back(row);
    }
    const auto evidence = ls2k::vision::DetectCrossExitEvidence(
        rows,
        Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
        params);
    Expect(!evidence.present,
           "a connected run bounded by two observed edges must remain a normal road row");
}

void TestMultipleConnectedRunsAreAmbiguous() {
    const ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
    rows.push_back(BoundedRoadRow(0.16F));
    for (float forward_m : {0.20F, 0.24F, 0.28F}) {
        auto row = ConnectedSingleBoundaryRow(forward_m);
        row.white_runs.push_back(row.white_runs.front());
        rows.push_back(row);
    }
    const auto evidence = ls2k::vision::DetectCrossExitEvidence(
        rows,
        Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
        params);
    Expect(!evidence.present,
           "multiple origin-connected runs must not choose a Cross opening arbitrarily");
}

void TestBlockedOriginToLastMidpointRejectsCross() {
    const ls2k::port::RuntimeParameters params{};
    const ls2k::port::CrossExitElementEvidence evidence =
        ls2k::vision::DetectCrossExitEvidence(
            BoundaryAbsentRows(3U),
            Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kBlocked),
            params);
    Expect(!evidence.present, "blocked origin-to-last-midpoint segment must reject cross");
    Expect(evidence.reason == "origin_to_cross_sample_midpoint_blocked",
           "blocked segment must expose the connectivity rejection reason");
}

void TestUnobservableOriginToLastMidpointRejectsCross() {
    const ls2k::port::RuntimeParameters params{};
    const ls2k::port::CrossExitElementEvidence evidence =
        ls2k::vision::DetectCrossExitEvidence(
            BoundaryAbsentRows(3U),
            Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kUnobservable),
            params);
    Expect(!evidence.present, "unobservable origin-to-last-midpoint segment must reject cross");
    Expect(evidence.reason == "origin_to_cross_sample_midpoint_unobservable",
           "unobservable segment must expose the observability rejection reason");
}

}  // namespace

int main() {
    try {
        TestTwoRowsRemainAbsent();
        TestThreeRowsRemainPresent();
        TestRightBoundaryExpansionWithLeftFovIsCross();
        TestLeftBoundaryExpansionWithRightFovIsCross();
        TestExpansionUsesBoundaryRelativeDistanceUnderRotation();
        TestSingleExpandedRowIsCurrentFrameCross();
        TestExpansionRequiresOppositeFov();
        TestNoTransitionWithoutConnectedWhiteRunIsNotOpen();
        TestConnectedCrossOpeningIgnoresUnrelatedRowBoundary();
        TestBoundedRoadLosingOneSideRemainsOrdinary();
        TestSingleBoundaryRunRemainsOrdinaryRoad();
        TestBoundaryBoundConnectedRunIsNotOpen();
        TestMultipleConnectedRunsAreAmbiguous();
        TestBlockedOriginToLastMidpointRejectsCross();
        TestUnobservableOriginToLastMidpointRejectsCross();
    } catch (const TestFailure& failure) {
        std::cerr << "cross_exit_element_evidence_test failed: " << failure.message << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "cross_exit_element_evidence_test passed\n";
    return EXIT_SUCCESS;
}
