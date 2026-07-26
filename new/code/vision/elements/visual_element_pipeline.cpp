#include "vision/elements/visual_element_pipeline.hpp"

#include "vision/elements/cross_exit_element_evidence.hpp"

namespace ls2k::vision {

VisualElementPipelineResult RunVisualElementPipeline(const VisualElementPipelineInput& input,
                                                     const port::RuntimeParameters& params) {
    VisualElementPipelineResult result{};
    const std::vector<BEVSimpleRowScan> empty_rows{};
    const std::vector<BEVSimpleRowScan>& rows =
        input.sparse_rows == nullptr ? empty_rows : *input.sparse_rows;

    result.evidence.cross_exit =
        DetectCrossExitEvidence(rows,
                                input.origin_to_cross_sample_midpoint_connectivity,
                                params);
    return result;
}

}  // namespace ls2k::vision
