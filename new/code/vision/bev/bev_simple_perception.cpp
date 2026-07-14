#include "vision/bev/bev_simple_perception.hpp"

// Simple BEV perception pipeline:
// frame view -> sparse BEV row facts -> reference path.

#include "port/perf_counter.hpp"
#include "vision/bev/bev_reference_path_builder.hpp"
#include "vision/bev/bev_image_segment_connectivity.hpp"
#include "vision/bev/bev_sparse_row_scanner.hpp"

namespace ls2k::vision {

const char* ToString(port::ReferenceMode mode) {
    switch (mode) {
        case port::ReferenceMode::kNone:
            return "none";
        case port::ReferenceMode::kIntervalCenter:
            return "interval_center";
        case port::ReferenceMode::kMlObservedBoundary:
            return "ml_observed_boundary";
        case port::ReferenceMode::kHoldLast:
            return "hold_last";
    }
    return "none";
}

const char* ToString(port::BEVPathPointSource source) {
    switch (source) {
        case port::BEVPathPointSource::kNone:
            return "none";
        case port::BEVPathPointSource::kIntervalCenter:
            return "interval_center";
        case port::BEVPathPointSource::kMlObservedBoundary:
            return "ml_observed_boundary";
        case port::BEVPathPointSource::kHold:
            return "hold";
    }
    return "none";
}

BEVSimplePerceptionResult RunBEVSimplePerception(const port::CameraPixelFrameView& frame,
                                                 const port::RuntimeParameters& params,
                                                 const BEVProjector& projector,
                                                 BEVSampleProjectionLut* lut) {
    BEVSimplePerceptionResult result{};
    BEVSampleProjectionLut local_lut{};
    BEVSampleProjectionLut& active_lut = lut == nullptr ? local_lut : *lut;
    {
        LS2K_PERF_SCOPE(port::PerfStage::kBevSimpleLut);
        if (!EnsureBEVSampleProjectionLut(active_lut, frame, params, projector)) {
            return result;
        }
    }

    {
        LS2K_PERF_SCOPE(port::PerfStage::kBevSimpleScanRows);
        result.rows = ScanSparseRows(frame, params, active_lut);
    }
    for (const BEVSimpleRowScan& row : result.rows) {
        result.boundary_jump_count += row.jumps.size();
        result.boundary_span_count += row.spans.size();
    }
    const BEVImageSegmentConnectivity connectivity(frame, projector, params.bev_boundary);
    result.road_path_facts =
        BuildConnectedRoadPathFacts(result.rows, params, connectivity);
    for (std::size_t index = 0; index < result.road_path_facts.center.size(); ++index) {
        result.reference_path.sampled_path[index].point.forward_m =
            params.bev_geometry.forward_samples_m[index];
        const BEVRoadPathPointFact& center = result.road_path_facts.center[index];
        if (!center.present) {
            continue;
        }
        port::BEVPathSample& sample = result.reference_path.sampled_path[index];
        result.reference_path.mode = port::ReferenceMode::kIntervalCenter;
        sample.present = true;
        sample.point = center.point;
        sample.confidence = center.confidence;
        sample.source = port::BEVPathPointSource::kIntervalCenter;
    }
    result.reference_mode = ToString(result.reference_path.mode);
    result.reference_source =
        result.reference_path.mode == port::ReferenceMode::kIntervalCenter ? "simple_interval_center" : "none";
    return result;
}

}  // namespace ls2k::vision
