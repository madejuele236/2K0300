#include "vision/elements/cross_straight_path_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "vision/bev/bev_reference_connectivity_filter.hpp"

namespace ls2k::vision {
namespace {

constexpr float kGeometryEpsilon = 1.0e-6F;

struct SelectedRun {
    const BEVSimpleRowScan* row = nullptr;
    const BEVWhiteRun* run = nullptr;
};

struct Line {
    bool valid = false;
    float slope = 0.0F;
    float intercept = 0.0F;

    float At(float forward_m) const {
        return slope * forward_m + intercept;
    }
};

struct Point {
    float forward_m = 0.0F;
    float lateral_m = 0.0F;
};

enum class CenterObservationMode {
    kNone,
    kBounded,
    kLeftFov,
    kRightFov,
};

struct Portal {
    float forward_m = 0.0F;
    float left_m = 0.0F;
    float guide_m = 0.0F;
    float right_m = 0.0F;
};

struct CorridorNode {
    Point point{};
    double guide_deviation = 0.0;
    double distance = 0.0;
    std::size_t previous = 0U;
    bool reached = false;
};

struct CenterGuide {
    Line entry{};
    Line exit{};
    float transition_begin_m = 0.0F;
    float transition_end_m = 0.0F;

    float At(float forward_m) const {
        if (forward_m <= transition_begin_m) {
            return entry.At(forward_m);
        }
        if (forward_m >= transition_end_m) {
            return exit.At(forward_m);
        }

        const float span_m = transition_end_m - transition_begin_m;
        const float t = (forward_m - transition_begin_m) / span_m;
        const float t2 = t * t;
        const float t3 = t2 * t;
        const float entry_lateral_m = entry.At(transition_begin_m);
        const float exit_lateral_m = exit.At(transition_end_m);
        return (2.0F * t3 - 3.0F * t2 + 1.0F) * entry_lateral_m +
               (t3 - 2.0F * t2 + t) * span_m * entry.slope +
               (-2.0F * t3 + 3.0F * t2) * exit_lateral_m +
               (t3 - t2) * span_m * exit.slope;
    }
};

CenterObservationMode EntryCenter(const SelectedRun& selected,
                                  float half_width_m,
                                  Point& center,
                                  Point& observed_boundary);
Line FitMetricLine(const std::vector<Point>& points);

port::VisualReferenceCandidate Absent(const char* reason) {
    port::VisualReferenceCandidate candidate{};
    candidate.kind = port::VisualReferenceCandidateKind::kCrossExit;
    candidate.source = "cross_straight";
    candidate.reason = reason;
    return candidate;
}

bool IntervalsOverlap(const BEVWhiteRun& lhs, const BEVWhiteRun& rhs) {
    return std::max(lhs.left_m, rhs.left_m) + kGeometryEpsilon <
           std::min(lhs.right_m, rhs.right_m);
}

float Distance(float forward_a, float lateral_a, float forward_b, float lateral_b) {
    return std::hypot(forward_b - forward_a, lateral_b - lateral_a);
}

bool SameEndpointContinues(const SelectedRun& previous,
                           const BEVSimpleRowScan& row,
                           const BEVWhiteRun& candidate,
                           float max_distance_m) {
    if (previous.row == nullptr || previous.run == nullptr) {
        return false;
    }
    const BEVWhiteRun& prior = *previous.run;
    if (prior.left_endpoint == BEVWhiteRunEndpointState::kBoundary &&
        candidate.left_endpoint == BEVWhiteRunEndpointState::kBoundary &&
        Distance(previous.row->forward_m,
                 prior.left_m,
                 row.forward_m,
                 candidate.left_m) <= max_distance_m) {
        return true;
    }
    return prior.right_endpoint == BEVWhiteRunEndpointState::kBoundary &&
           candidate.right_endpoint == BEVWhiteRunEndpointState::kBoundary &&
           Distance(previous.row->forward_m,
                    prior.right_m,
                    row.forward_m,
                    candidate.right_m) <= max_distance_m;
}

const BEVWhiteRun* UniqueOriginConnectedRun(const BEVSimpleRowScan& row,
                                            bool& ambiguous) {
    const BEVWhiteRun* selected = nullptr;
    for (const BEVWhiteRun& run : row.white_runs) {
        if (run.origin_connectivity != BEVWhiteRunOriginConnectivity::kConnected) {
            continue;
        }
        if (selected != nullptr) {
            ambiguous = true;
            return nullptr;
        }
        selected = &run;
    }
    return selected;
}

const BEVWhiteRun* UniqueContinuation(const SelectedRun& previous,
                                      const BEVSimpleRowScan& row,
                                      float max_distance_m,
                                      float half_width_m,
                                      const Line& straight_guidance,
                                      bool& ambiguous) {
    std::vector<const BEVWhiteRun*> candidates{};
    for (const BEVWhiteRun& run : row.white_runs) {
        if (IntervalsOverlap(*previous.run, run)) {
            candidates.push_back(&run);
        }
    }
    if (candidates.empty()) {
        for (const BEVWhiteRun& run : row.white_runs) {
            if (SameEndpointContinues(previous, row, run, max_distance_m)) {
                candidates.push_back(&run);
            }
        }
    }
    if (candidates.empty()) {
        return nullptr;
    }
    if (candidates.size() == 1U) {
        return candidates.front();
    }

    const float target_lateral_m = straight_guidance.At(row.forward_m);
    const BEVWhiteRun* selected = nullptr;
    float best_distance_m = 0.0F;
    bool best_tied = false;
    for (const BEVWhiteRun* candidate : candidates) {
        float center_m = 0.5F * (candidate->left_m + candidate->right_m);
        if (candidate->left_endpoint == BEVWhiteRunEndpointState::kFovEdge &&
            candidate->right_endpoint == BEVWhiteRunEndpointState::kBoundary) {
            center_m = candidate->right_m - half_width_m;
        } else if (candidate->left_endpoint == BEVWhiteRunEndpointState::kBoundary &&
                   candidate->right_endpoint == BEVWhiteRunEndpointState::kFovEdge) {
            center_m = candidate->left_m + half_width_m;
        }
        const float distance_m = std::abs(center_m - target_lateral_m);
        if (selected == nullptr || distance_m < best_distance_m - kGeometryEpsilon) {
            selected = candidate;
            best_distance_m = distance_m;
            best_tied = false;
        } else if (std::abs(distance_m - best_distance_m) <= kGeometryEpsilon) {
            best_tied = true;
        }
    }
    ambiguous = best_tied;
    return best_tied ? nullptr : selected;
}

bool SelectRoute(const std::vector<BEVSimpleRowScan>& rows,
                 float max_distance_m,
                 float half_width_m,
                 const Line& straight_guidance,
                 std::vector<SelectedRun>& route,
                 const char*& failure_reason) {
    bool origin_ambiguous_seen = false;
    for (const BEVSimpleRowScan& row : rows) {
        if (!row.valid || row.white_runs.empty()) {
            continue;
        }

        bool ambiguous = false;
        const BEVWhiteRun* selected = nullptr;
        if (route.empty()) {
            selected = UniqueOriginConnectedRun(row, ambiguous);
            if (ambiguous) {
                origin_ambiguous_seen = true;
                continue;
            }
        } else {
            if (row.forward_m - route.back().row->forward_m >
                max_distance_m + kGeometryEpsilon) {
                break;
            }
            selected = UniqueContinuation(route.back(),
                                          row,
                                          max_distance_m,
                                          half_width_m,
                                          straight_guidance,
                                          ambiguous);
            if (ambiguous) {
                continue;
            }
        }
        if (selected == nullptr) {
            continue;
        }
        route.push_back(SelectedRun{&row, selected});
    }
    if (route.empty()) {
        failure_reason = origin_ambiguous_seen
                             ? "cross_route_origin_ambiguous"
                             : "cross_route_origin_absent";
        return false;
    }
    return true;
}

float ProjectIntoInterval(float value, float minimum, float maximum) {
    return std::max(minimum, std::min(value, maximum));
}

CenterObservationMode EntryCenter(const SelectedRun& selected,
                                  float half_width_m,
                                  Point& center,
                                  Point& observed_boundary) {
    const BEVWhiteRun& run = *selected.run;
    center.forward_m = selected.row->forward_m;
    observed_boundary.forward_m = selected.row->forward_m;
    if (run.left_endpoint == BEVWhiteRunEndpointState::kBoundary &&
        run.right_endpoint == BEVWhiteRunEndpointState::kBoundary) {
        center.lateral_m = 0.5F * (run.left_m + run.right_m);
        observed_boundary.lateral_m = center.lateral_m;
        return CenterObservationMode::kBounded;
    }
    if (run.left_endpoint == BEVWhiteRunEndpointState::kFovEdge &&
        run.right_endpoint == BEVWhiteRunEndpointState::kBoundary) {
        observed_boundary.lateral_m = run.right_m;
        center.lateral_m = ProjectIntoInterval(run.right_m - half_width_m,
                                               run.left_m,
                                               run.right_m);
        return CenterObservationMode::kLeftFov;
    }
    if (run.left_endpoint == BEVWhiteRunEndpointState::kBoundary &&
        run.right_endpoint == BEVWhiteRunEndpointState::kFovEdge) {
        observed_boundary.lateral_m = run.left_m;
        center.lateral_m = ProjectIntoInterval(run.left_m + half_width_m,
                                               run.left_m,
                                               run.right_m);
        return CenterObservationMode::kRightFov;
    }
    return CenterObservationMode::kNone;
}

CenterObservationMode ExitCenter(const SelectedRun& selected,
                                 float half_width_m,
                                 Point& center) {
    const BEVWhiteRun& run = *selected.run;
    center.forward_m = selected.row->forward_m;
    if (run.left_endpoint == BEVWhiteRunEndpointState::kBoundary &&
        run.right_endpoint == BEVWhiteRunEndpointState::kBoundary) {
        center.lateral_m = 0.5F * (run.left_m + run.right_m);
        return CenterObservationMode::kBounded;
    }
    if (run.left_endpoint == BEVWhiteRunEndpointState::kFovEdge &&
        run.right_endpoint == BEVWhiteRunEndpointState::kBoundary) {
        center.lateral_m = ProjectIntoInterval(run.right_m - half_width_m,
                                               run.left_m,
                                               run.right_m);
        return CenterObservationMode::kLeftFov;
    }
    if (run.left_endpoint == BEVWhiteRunEndpointState::kBoundary &&
        run.right_endpoint == BEVWhiteRunEndpointState::kFovEdge) {
        center.lateral_m = ProjectIntoInterval(run.left_m + half_width_m,
                                               run.left_m,
                                               run.right_m);
        return CenterObservationMode::kRightFov;
    }
    return CenterObservationMode::kNone;
}

Line FitMetricLine(const std::vector<Point>& points) {
    Line line{};
    if (points.size() < 2U) {
        return line;
    }

    const auto median = [](std::vector<float> values) {
        const std::size_t middle = values.size() / 2U;
        std::nth_element(values.begin(), values.begin() + middle, values.end());
        if (values.size() % 2U != 0U) {
            return values[middle];
        }
        const float upper = values[middle];
        std::nth_element(values.begin(), values.begin() + middle - 1U, values.end());
        return 0.5F * (values[middle - 1U] + upper);
    };

    // Each observation first votes with its median slope to every other
    // observation. Taking the median vote makes one bad row unable to rotate
    // the road axis fitted from the remaining rows.
    std::vector<float> slope_votes{};
    slope_votes.reserve(points.size());
    for (std::size_t index = 0U; index < points.size(); ++index) {
        std::vector<float> slopes{};
        slopes.reserve(points.size() - 1U);
        for (std::size_t other = 0U; other < points.size(); ++other) {
            if (other == index) {
                continue;
            }
            const float forward_delta = points[other].forward_m - points[index].forward_m;
            if (std::abs(forward_delta) <= kGeometryEpsilon) {
                continue;
            }
            slopes.push_back((points[other].lateral_m - points[index].lateral_m) /
                             forward_delta);
        }
        if (!slopes.empty()) {
            slope_votes.push_back(median(std::move(slopes)));
        }
    }
    if (slope_votes.size() < 2U) {
        return line;
    }
    line.slope = median(std::move(slope_votes));

    std::vector<float> intercepts{};
    intercepts.reserve(points.size());
    for (const Point& point : points) {
        intercepts.push_back(point.lateral_m - line.slope * point.forward_m);
    }
    line.intercept = median(std::move(intercepts));
    line.valid = std::isfinite(line.slope) && std::isfinite(line.intercept);
    return line;
}

bool FindExitSegment(const std::vector<SelectedRun>& route,
                     float half_width_m,
                     float max_distance_m,
                     float entry_end_m,
                     bool bounded_only,
                     std::vector<Point>& best_observations,
                     std::size_t& best_route_end) {
    best_observations.clear();
    best_route_end = 0U;
    float best_forward_span_m = -1.0F;
    std::vector<Point> current{};
    std::size_t current_route_end = 0U;
    const auto consider_current = [&]() {
        if (current.size() < 2U) {
            return;
        }
        const float forward_span_m = current.back().forward_m - current.front().forward_m;
        if (forward_span_m > best_forward_span_m + kGeometryEpsilon ||
            (std::abs(forward_span_m - best_forward_span_m) <= kGeometryEpsilon &&
             current_route_end > best_route_end)) {
            best_observations = current;
            best_route_end = current_route_end;
            best_forward_span_m = forward_span_m;
        }
    };

    for (std::size_t index = 0U; index < route.size(); ++index) {
        Point observation{};
        const CenterObservationMode mode = ExitCenter(route[index], half_width_m, observation);
        const bool valid = observation.forward_m > entry_end_m + kGeometryEpsilon &&
                           mode != CenterObservationMode::kNone &&
                           (!bounded_only || mode == CenterObservationMode::kBounded);
        if (!valid) {
            // This row contributes no exit-line fact. It does not erase the
            // surrounding valid facts; their actual metric gap decides continuity.
            continue;
        }

        if (!current.empty() &&
            Distance(current.back().forward_m,
                     current.back().lateral_m,
                     observation.forward_m,
                     observation.lateral_m) > max_distance_m) {
            consider_current();
            current.clear();
        }
        current.push_back(observation);
        current_route_end = index + 1U;
    }
    consider_current();
    return best_observations.size() >= 2U;
}

bool BetterCorridorCost(double guide_deviation,
                        double distance,
                        const CorridorNode& current) {
    constexpr double kCostEpsilon = 1.0e-12;
    if (!current.reached ||
        guide_deviation < current.guide_deviation - kCostEpsilon) {
        return true;
    }
    return std::abs(guide_deviation - current.guide_deviation) <= kCostEpsilon &&
           distance < current.distance;
}

bool BuildCenterGuidedCorridorPath(const std::vector<Portal>& portals,
                                   const BEVSegmentConnectivityQuery& connectivity,
                                   std::vector<Point>& corners) {
    if (portals.size() < 2U) {
        return false;
    }

    std::vector<CorridorNode> nodes{};
    std::vector<std::size_t> portal_begin{};
    std::vector<std::size_t> portal_end{};
    constexpr std::size_t kPortalNodeCount = 3U;
    nodes.reserve(portals.size() * kPortalNodeCount);
    portal_begin.reserve(portals.size());
    portal_end.reserve(portals.size());
    for (std::size_t index = 0U; index < portals.size(); ++index) {
        const Portal& portal = portals[index];
        portal_begin.push_back(nodes.size());
        const float lateral_candidates[kPortalNodeCount] = {
            portal.left_m, portal.guide_m, portal.right_m};
        for (const float lateral_m : lateral_candidates) {
            if (nodes.size() > portal_begin.back() &&
                std::abs(nodes.back().point.lateral_m - lateral_m) <= kGeometryEpsilon) {
                continue;
            }
            nodes.push_back(CorridorNode{Point{portal.forward_m, lateral_m}});
        }
        portal_end.push_back(nodes.size());
    }
    const std::size_t start_index = portal_begin.front();
    nodes[start_index].distance = 0.0;
    nodes[start_index].reached = true;

    std::size_t last_reachable_portal = 0U;
    for (std::size_t portal_index = 1U; portal_index < portals.size(); ++portal_index) {
        bool portal_reached = false;
        for (std::size_t to_index = portal_begin[portal_index];
             to_index < portal_end[portal_index];
             ++to_index) {
            CorridorNode& to = nodes[to_index];
            for (std::size_t from_index = portal_begin[last_reachable_portal];
                 from_index < portal_end[last_reachable_portal];
                 ++from_index) {
                const CorridorNode& from = nodes[from_index];
                if (!from.reached) {
                    continue;
                }
                const port::BEVPoint from_bev{from.point.forward_m,
                                              from.point.lateral_m};
                const port::BEVPoint to_bev{to.point.forward_m,
                                            to.point.lateral_m};
                if (connectivity.Evaluate(from_bev,
                                          to_bev,
                                          BEVSegmentVisibilityPolicy::kRequireFullSegment)
                        .status != BEVSegmentConnectivityStatus::kConnected) {
                    continue;
                }
                const double candidate_distance =
                    from.distance + Distance(from.point.forward_m,
                                             from.point.lateral_m,
                                             to.point.forward_m,
                                             to.point.lateral_m);
                const double forward_span_m =
                    static_cast<double>(to.point.forward_m - from.point.forward_m);
                const double from_deviation_m =
                    std::abs(static_cast<double>(from.point.lateral_m -
                                                 portals[last_reachable_portal].guide_m));
                const double to_deviation_m =
                    std::abs(static_cast<double>(to.point.lateral_m -
                                                 portals[portal_index].guide_m));
                const double candidate_guide_deviation =
                    from.guide_deviation +
                    0.5 * (from_deviation_m + to_deviation_m) * forward_span_m;
                // Centering is the route objective. Length only resolves routes
                // with the same meter-integrated deviation from that center guide.
                if (BetterCorridorCost(candidate_guide_deviation,
                                       candidate_distance,
                                       to)) {
                    to.reached = true;
                    to.guide_deviation = candidate_guide_deviation;
                    to.distance = candidate_distance;
                    to.previous = from_index;
                    portal_reached = true;
                }
            }
        }
        if (portal_reached) {
            last_reachable_portal = portal_index;
        }
    }

    if (last_reachable_portal == 0U) {
        return false;
    }
    std::size_t goal_index = portal_begin[last_reachable_portal];
    for (std::size_t index = portal_begin[last_reachable_portal];
         index < portal_end[last_reachable_portal];
         ++index) {
        if (nodes[index].reached &&
            BetterCorridorCost(nodes[index].guide_deviation,
                               nodes[index].distance,
                               nodes[goal_index])) {
            goal_index = index;
        }
    }
    if (!nodes[goal_index].reached) {
        return false;
    }
    std::vector<Point> reverse{};
    for (std::size_t index = goal_index;; index = nodes[index].previous) {
        reverse.push_back(nodes[index].point);
        if (index == start_index) {
            break;
        }
    }
    corners.assign(reverse.rbegin(), reverse.rend());
    return true;
}

bool ComposeCorridorToExit(const std::vector<SelectedRun>& route,
                           const std::vector<Point>& entry_centers,
                           const std::vector<Point>& exit_centers,
                           std::size_t route_end,
                           const port::RuntimeParameters& params,
                           const BEVSegmentConnectivityQuery& connectivity,
                           std::vector<Point>& corridor_corners,
                           const char*& failure_reason) {
    const Line entry_line = FitMetricLine(entry_centers);
    const Line exit_line = FitMetricLine(exit_centers);
    if (!entry_line.valid || !exit_line.valid) {
        failure_reason = "cross_line_fit_invalid";
        return false;
    }

    const float entry_end_m = entry_centers.back().forward_m;
    const float exit_start_m = exit_centers.front().forward_m;
    if (!(exit_start_m > entry_end_m + kGeometryEpsilon)) {
        failure_reason = "cross_transition_span_absent";
        return false;
    }
    const CenterGuide center_guide{
        entry_line, exit_line, entry_end_m, exit_start_m};

    std::vector<Portal> portals{};
    portals.reserve(route_end);
    const float boundary_inset_m = 0.5F * params.bev_geometry.lateral_step_m;
    for (std::size_t index = 0U; index < route_end; ++index) {
        const SelectedRun& selected = route[index];
        const BEVWhiteRun& run = *selected.run;
        Portal portal{selected.row->forward_m, run.left_m, 0.0F, run.right_m};
        if (run.left_endpoint == BEVWhiteRunEndpointState::kBoundary) {
            portal.left_m += boundary_inset_m;
        }
        if (run.right_endpoint == BEVWhiteRunEndpointState::kBoundary) {
            portal.right_m -= boundary_inset_m;
        }
        if (portal.left_m > portal.right_m) {
            continue;
        }
        portal.guide_m = ProjectIntoInterval(center_guide.At(portal.forward_m),
                                             portal.left_m,
                                             portal.right_m);
        if (portals.empty()) {
            portal.left_m = portal.guide_m;
            portal.right_m = portal.guide_m;
        }
        portals.push_back(portal);
    }

    if (!BuildCenterGuidedCorridorPath(portals, connectivity, corridor_corners)) {
        failure_reason = "cross_route_corridor_disconnected";
        return false;
    }
    return true;
}

float InterpolateCorridorPath(const std::vector<Point>& corners, float forward_m) {
    if (forward_m <= corners.front().forward_m) {
        return corners.front().lateral_m;
    }
    for (std::size_t index = 1U; index < corners.size(); ++index) {
        if (forward_m > corners[index].forward_m) {
            continue;
        }
        const Point& from = corners[index - 1U];
        const Point& to = corners[index];
        const float t = (forward_m - from.forward_m) / (to.forward_m - from.forward_m);
        return from.lateral_m + t * (to.lateral_m - from.lateral_m);
    }
    return corners.back().lateral_m;
}

}  // namespace

port::VisualReferenceCandidate BuildCrossStraightPathCandidate(
    const std::vector<BEVSimpleRowScan>& rows,
    const port::CrossExitElementEvidence& evidence,
    const port::RuntimeParameters& params,
    const BEVSegmentConnectivityQuery& connectivity) {
    if (!evidence.present) {
        return Absent("cross_not_present");
    }

    const float max_distance_m =
        std::max(0.0F, params.bev_geometry.boundary_trace_max_adjacent_distance_m);
    const float half_width_m = params.bev_geometry.nominal_road_half_width_m;
    std::vector<Point> guidance_centers{};
    Point guidance_previous_boundary{};
    bool have_guidance_boundary = false;
    CenterObservationMode guidance_mode = CenterObservationMode::kNone;
    for (const BEVSimpleRowScan& row : rows) {
        bool ambiguous = false;
        const BEVWhiteRun* run = UniqueOriginConnectedRun(row, ambiguous);
        if (ambiguous) {
            continue;
        }
        if (run == nullptr) {
            continue;
        }
        Point center{};
        Point boundary{};
        const SelectedRun selected{&row, run};
        const CenterObservationMode mode =
            EntryCenter(selected, half_width_m, center, boundary);
        if (mode == CenterObservationMode::kNone) {
            continue;
        }
        if (guidance_mode == CenterObservationMode::kNone) {
            guidance_mode = mode;
        } else if (mode != guidance_mode) {
            break;
        }
        if (have_guidance_boundary &&
            Distance(guidance_previous_boundary.forward_m,
                     guidance_previous_boundary.lateral_m,
                     boundary.forward_m,
                     boundary.lateral_m) > max_distance_m) {
            break;
        }
        guidance_centers.push_back(center);
        guidance_previous_boundary = boundary;
        have_guidance_boundary = true;
    }
    const Line straight_guidance = FitMetricLine(guidance_centers);
    if (!straight_guidance.valid) {
        return Absent("cross_entry_line_absent");
    }
    std::vector<SelectedRun> route{};
    route.reserve(rows.size());
    const char* failure_reason = "none";
    if (!SelectRoute(rows,
                     max_distance_m,
                     half_width_m,
                     straight_guidance,
                     route,
                     failure_reason)) {
        return Absent(failure_reason);
    }

    std::vector<Point> entry_centers{};
    entry_centers.reserve(route.size());
    Point previous_boundary{};
    bool have_previous_boundary = false;
    CenterObservationMode entry_mode = CenterObservationMode::kNone;
    for (const SelectedRun& selected : route) {
        Point center{};
        Point boundary{};
        const CenterObservationMode mode =
            EntryCenter(selected, half_width_m, center, boundary);
        if (mode == CenterObservationMode::kNone) {
            continue;
        }
        if (entry_mode == CenterObservationMode::kNone) {
            entry_mode = mode;
        } else if (mode != entry_mode) {
            break;
        }
        if (have_previous_boundary &&
            Distance(previous_boundary.forward_m,
                     previous_boundary.lateral_m,
                     boundary.forward_m,
                     boundary.lateral_m) > max_distance_m) {
            break;
        }
        entry_centers.push_back(center);
        previous_boundary = boundary;
        have_previous_boundary = true;
    }
    if (entry_centers.size() < 2U) {
        return Absent("cross_entry_line_absent");
    }

    std::vector<Point> exit_observations{};
    std::size_t exit_route_end = 0U;
    std::size_t route_end = 0U;
    std::vector<Point> corridor_corners{};
    const char* composition_failure = "cross_exit_line_absent";
    // A two-boundary center line is direct exit geometry. A one-sided FOV
    // estimate is considered only when that geometry cannot form a connected route.
    const bool bounded_exit_found = FindExitSegment(route,
                                                    half_width_m,
                                                    max_distance_m,
                                                    entry_centers.back().forward_m,
                                                    true,
                                                    exit_observations,
                                                    exit_route_end);
    if (bounded_exit_found &&
        ComposeCorridorToExit(route,
                              entry_centers,
                              exit_observations,
                              exit_route_end,
                              params,
                              connectivity,
                              corridor_corners,
                              composition_failure)) {
        route_end = exit_route_end;
    }
    if (route_end == 0U) {
        if (!FindExitSegment(route,
                             half_width_m,
                             max_distance_m,
                             entry_centers.back().forward_m,
                             false,
                             exit_observations,
                             exit_route_end)) {
            return Absent(bounded_exit_found ? composition_failure
                                             : "cross_exit_line_absent");
        }
        if (!ComposeCorridorToExit(route,
                                   entry_centers,
                                   exit_observations,
                                   exit_route_end,
                                   params,
                                   connectivity,
                                   corridor_corners,
                                   composition_failure)) {
            return Absent(composition_failure);
        }
        route_end = exit_route_end;
    }
    route.resize(route_end);

    port::VisualReferenceCandidate candidate{};
    candidate.present = true;
    candidate.kind = port::VisualReferenceCandidateKind::kCrossExit;
    candidate.source = "cross_straight";
    candidate.reason = "cross_entry_route_exit_path";
    candidate.reference_path.mode = port::ReferenceMode::kIntervalCenter;

    std::size_t output_index = 0U;
    for (const SelectedRun& selected : route) {
        if (output_index >= candidate.reference_path.sampled_path.size()) {
            break;
        }
        const float forward_m = selected.row->forward_m;
        const float lateral_m = InterpolateCorridorPath(corridor_corners, forward_m);
        port::BEVPathSample& sample = candidate.reference_path.sampled_path[output_index++];
        sample.present = true;
        sample.point = {forward_m, lateral_m};
        sample.confidence = 1.0F;
        sample.source = port::BEVPathPointSource::kIntervalCenter;
    }

    if (KeepConnectedReferenceSamples(candidate.reference_path, connectivity) < 2U) {
        candidate.present = false;
        candidate.reason = "cross_path_connectivity_insufficient";
        return candidate;
    }
    return candidate;
}

}  // namespace ls2k::vision
