#include "vision/elements/visual_element_pipeline.hpp"

#include "port/perf_counter.hpp"
#include "vision/elements/cross_exit_element_evidence.hpp"
#include "vision/elements/cross_straight_path_planner.hpp"
#include "vision/elements/zebra_element_evidence.hpp"

namespace ls2k::vision {

VisualElementPipelineResult RunVisualElementPipeline(const VisualElementPipelineInput& input,
                                                     const port::RuntimeParameters& params) {
    VisualElementPipelineResult result{};
    const std::vector<BEVSimpleRowScan> empty_rows{};
    const std::vector<BEVSimpleRowScan>& rows =
        input.sparse_rows == nullptr ? empty_rows : *input.sparse_rows;

    {
        LS2K_PERF_SCOPE(port::PerfStage::kCrossExitDetection);
        result.evidence.cross_exit =
            DetectCrossExitEvidence(rows,
                                    input.origin_to_cross_sample_midpoint_connectivity,
                                    params);
    }
    {
        LS2K_PERF_SCOPE(port::PerfStage::kZebraDetection);
        result.evidence.records.push_back(DetectZebraEvidence(rows, params));
    }
    if (input.segment_connectivity != nullptr) {
        LS2K_PERF_SCOPE(port::PerfStage::kCrossStraightPlanning);
        result.cross_candidate =
            BuildCrossStraightPathCandidate(rows,
                                            result.evidence.cross_exit,
                                            params,
                                            *input.segment_connectivity);
    } else {
        result.cross_candidate.kind = port::VisualReferenceCandidateKind::kCrossExit;
        result.cross_candidate.source = "cross_straight";
        result.cross_candidate.reason = "segment_connectivity_absent";
    }
    return result;
}

}  // namespace ls2k::vision
