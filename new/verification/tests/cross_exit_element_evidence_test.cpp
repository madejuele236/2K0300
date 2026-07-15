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

void TestBlockedOriginToLastMidpointRejectsCross() {
    const ls2k::port::RuntimeParameters params{};
    const ls2k::port::CrossExitElementEvidence evidence =
        ls2k::vision::DetectCrossExitEvidence(
            BoundaryAbsentRows(3U),
            Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kBlocked),
            params);
    Expect(!evidence.present, "blocked origin-to-last-midpoint segment must reject cross");
    Expect(evidence.reason == "origin_to_last_midpoint_blocked",
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
    Expect(evidence.reason == "origin_to_last_midpoint_unobservable",
           "unobservable segment must expose the observability rejection reason");
}

void TestCandidateStillCopiesLineReference() {
    const ls2k::port::RuntimeParameters params{};
    const ls2k::port::CrossExitElementEvidence evidence =
        ls2k::vision::DetectCrossExitEvidence(
            BoundaryAbsentRows(3U),
            Connectivity(ls2k::vision::BEVSegmentConnectivityStatus::kConnected),
            params);
    ls2k::port::VisualReferenceCandidate line{};
    line.present = true;
    line.kind = ls2k::port::VisualReferenceCandidateKind::kLine;
    line.reference_path.mode = ls2k::port::ReferenceMode::kIntervalCenter;
    line.reference_path.sampled_path[0].present = true;
    line.reference_path.sampled_path[0].point.forward_m = 0.1F;
    line.reference_path.sampled_path[0].point.lateral_m = 0.02F;

    ls2k::port::VisualElementCandidateSummary summary{};
    const ls2k::port::VisualReferenceCandidate candidate =
        ls2k::vision::BuildCrossExitVisualReferenceCandidate(evidence, line, params, summary);

    Expect(candidate.present, "present cross evidence with a valid line must still build a candidate");
    Expect(candidate.kind == ls2k::port::VisualReferenceCandidateKind::kCrossExit,
           "built candidate must remain a cross-exit candidate");
    Expect(candidate.reference_path.sampled_path[0].point.lateral_m == 0.02F,
           "cross candidate must still copy the line reference path");
    Expect(summary.included_in_arbitration,
           "default-enabled cross takeover must still include the candidate in arbitration");
}

}  // namespace

int main() {
    try {
        TestTwoRowsRemainAbsent();
        TestThreeRowsRemainPresent();
        TestBlockedOriginToLastMidpointRejectsCross();
        TestUnobservableOriginToLastMidpointRejectsCross();
        TestCandidateStillCopiesLineReference();
    } catch (const TestFailure& failure) {
        std::cerr << "cross_exit_element_evidence_test failed: " << failure.message << "\n";
        return EXIT_FAILURE;
    }
    std::cout << "cross_exit_element_evidence_test passed\n";
    return EXIT_SUCCESS;
}
