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
        case port::ReferenceMode::kMlBoundaryOffset:
            return "ml_boundary_offset";
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
        case port::BEVPathPointSource::kMlBoundaryOffset:
            return "ml_boundary_offset";
        case port::BEVPathPointSource::kHold:
            return "hold";
    }
    return "none";
}

BEVSimplePerceptionResult RunBEVSimplePerception(const port::CameraPixelFrameView& frame,
                                                 const port::BinaryModelState& binary_model,
                                                 const port::RuntimeParameters& params,
                                                 const BEVProjector& projector,
                                                 BEVSampleProjectionLut* lut) {
    BEVSimplePerceptionResult result{};
    result.binary_model = binary_model;
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
        result.rows = ScanSparseRows(frame, binary_model, params, active_lut);
    }
    for (const BEVSimpleRowScan& row : result.rows) {
        result.boundary_jump_count += row.jumps.size();
        result.boundary_span_count += row.spans.size();
    }
    {
        LS2K_PERF_SCOPE(port::PerfStage::kBevSimpleConnectivity);
        const BEVImageSegmentConnectivity connectivity(frame, projector, binary_model);
        for (BEVSimpleRowScan& row : result.rows) {
            for (BEVWhiteRun& run : row.white_runs) {
                const BEVSegmentConnectivityResult status =
                    connectivity.Evaluate({0.0F, 0.0F},
                                          {run.forward_m, 0.5F * (run.left_m + run.right_m)},
                                          BEVSegmentVisibilityPolicy::kAllowFromEndpointClip);
                switch (status.status) {
                    case BEVSegmentConnectivityStatus::kConnected:
                        run.origin_connectivity = BEVWhiteRunOriginConnectivity::kConnected;
                        break;
                    case BEVSegmentConnectivityStatus::kBlocked:
                        run.origin_connectivity = BEVWhiteRunOriginConnectivity::kBlocked;
                        break;
                    case BEVSegmentConnectivityStatus::kUnobservable:
                        run.origin_connectivity = BEVWhiteRunOriginConnectivity::kUnobservable;
                        break;
                }
            }
        }
        const std::size_t cross_sample_index =
            static_cast<std::size_t>(params.bev_element.cross_connectivity_sample_index);
        result.origin_to_cross_sample_midpoint_connectivity =
            connectivity.Evaluate(
                {0.0F, 0.0F},
                {params.bev_geometry.forward_samples_m[cross_sample_index], 0.0F},
                BEVSegmentVisibilityPolicy::kAllowFromEndpointClip);
        result.road_path_facts =
            BuildConnectedRoadPathFacts(result.rows, params, connectivity);
    }
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
