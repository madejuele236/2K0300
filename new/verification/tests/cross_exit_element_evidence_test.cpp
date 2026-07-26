#include <cstdlib>
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
    return row;
}

std::vector<ls2k::vision::BEVSimpleRowScan> BoundaryAbsentRows(std::size_t count) {
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
    for (std::size_t index = 0; index < count; ++index) {
        rows.push_back(BoundaryAbsentRow(0.1F + 0.1F * static_cast<float>(index)));
    }
    return rows;
}

ls2k::vision::BEVSimpleRowScan ConnectedFovOpenRow(float forward_m) {
    ls2k::vision::BEVSimpleRowScan row = BoundaryAbsentRow(forward_m);
    row.jumps.push_back({});
    ls2k::vision::BEVWhiteRun run{};
    run.left_endpoint = ls2k::vision::BEVWhiteRunEndpointState::kFovEdge;
    run.right_endpoint = ls2k::vision::BEVWhiteRunEndpointState::kBoundary;
    run.origin_connectivity =
        ls2k::vision::BEVWhiteRunOriginConnectivity::kConnected;
    row.white_runs.push_back(run);
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
    Expect(evidence.reason == "present", "three-row evidence must retain the present reason");
}

void TestConnectedFovOpeningIgnoresUnrelatedRowBoundary() {
    const ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
    rows.push_back(ConnectedFovOpenRow(0.20F));
    rows.push_back(ConnectedFovOpenRow(0.24F));
    rows.push_back(ConnectedFovOpenRow(0.28F));
    const auto evidence = ls2k::vision::DetectCrossExitEvidence(
        rows,
        Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
        params);
    Expect(evidence.present,
           "the unique origin-connected run reaching FOV must define an open Cross row");
}

void TestBoundaryBoundConnectedRunIsNotOpen() {
    const ls2k::port::RuntimeParameters params{};
    std::vector<ls2k::vision::BEVSimpleRowScan> rows{};
    for (float forward_m : {0.20F, 0.24F, 0.28F}) {
        auto row = ConnectedFovOpenRow(forward_m);
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
    for (float forward_m : {0.20F, 0.24F, 0.28F}) {
        auto row = ConnectedFovOpenRow(forward_m);
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
        TestConnectedFovOpeningIgnoresUnrelatedRowBoundary();
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
