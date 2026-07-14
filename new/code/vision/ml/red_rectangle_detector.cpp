#include "vision/ml/red_rectangle_detector.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "vision/image/color_sampler.hpp"

namespace ls2k::vision::ml {
namespace {

struct Cell {
    float forward = 0.0F;
    float lateral = 0.0F;
    bool red = false;
    bool visited = false;
};

struct Point {
    float forward = 0.0F;
    float lateral = 0.0F;
};

float Cross(const Point& origin, const Point& a, const Point& b) {
    return (a.forward - origin.forward) * (b.lateral - origin.lateral) -
           (a.lateral - origin.lateral) * (b.forward - origin.forward);
}

std::vector<Point> ConvexHull(std::vector<Point> points) {
    std::sort(points.begin(), points.end(), [](const Point& a, const Point& b) {
        return a.forward < b.forward ||
               (a.forward == b.forward && a.lateral < b.lateral);
    });
    points.erase(std::unique(points.begin(), points.end(), [](const Point& a, const Point& b) {
        return a.forward == b.forward && a.lateral == b.lateral;
    }), points.end());
    if (points.size() <= 2U) return points;
    std::vector<Point> hull;
    hull.reserve(points.size() * 2U);
    for (const Point& point : points) {
        while (hull.size() >= 2U &&
               Cross(hull[hull.size() - 2U], hull.back(), point) <= 0.0F) {
            hull.pop_back();
        }
        hull.push_back(point);
    }
    const std::size_t lower_size = hull.size();
    for (auto iterator = points.rbegin() + 1; iterator != points.rend(); ++iterator) {
        while (hull.size() > lower_size &&
               Cross(hull[hull.size() - 2U], hull.back(), *iterator) <= 0.0F) {
            hull.pop_back();
        }
        hull.push_back(*iterator);
    }
    hull.pop_back();
    return hull;
}

float PolygonArea(const std::vector<Point>& points) {
    if (points.size() < 3U) return 0.0F;
    float twice_area = 0.0F;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const Point& a = points[index];
        const Point& b = points[(index + 1U) % points.size()];
        twice_area += a.forward * b.lateral - b.forward * a.lateral;
    }
    return std::fabs(twice_area) * 0.5F;
}

bool InRange(std::uint8_t value, int minimum, int maximum) {
    return value >= minimum && value <= maximum;
}

float UnitScore(float error, float tolerance) {
    return std::clamp(1.0F - error / tolerance, 0.0F, 1.0F);
}

port::MlOrientedRectangle MeasureComponent(const std::vector<int>& indexes,
                                            const std::vector<Cell>& grid,
                                            float forward_step,
                                            float lateral_step,
                                            const port::MlRoiParameters& params) {
    port::MlOrientedRectangle out{};
    if (indexes.size() < static_cast<std::size_t>(params.min_component_cells)) return out;
    std::vector<Point> cell_corners;
    cell_corners.reserve(indexes.size() * 4U);
    const float half_forward_step = forward_step * 0.5F;
    const float half_lateral_step = lateral_step * 0.5F;
    for (int index : indexes) {
        const Cell& cell = grid[static_cast<std::size_t>(index)];
        cell_corners.push_back({cell.forward - half_forward_step, cell.lateral - half_lateral_step});
        cell_corners.push_back({cell.forward - half_forward_step, cell.lateral + half_lateral_step});
        cell_corners.push_back({cell.forward + half_forward_step, cell.lateral - half_lateral_step});
        cell_corners.push_back({cell.forward + half_forward_step, cell.lateral + half_lateral_step});
    }
    const std::vector<Point> hull = ConvexHull(std::move(cell_corners));
    if (hull.size() < 3U) return out;
    float best_area = std::numeric_limits<float>::max();
    float best_min_a = 0.0F, best_max_a = 0.0F;
    float best_min_b = 0.0F, best_max_b = 0.0F;
    float axis_f = 0.0F, axis_l = 0.0F;
    for (std::size_t edge = 0; edge < hull.size(); ++edge) {
        const Point& first = hull[edge];
        const Point& second = hull[(edge + 1U) % hull.size()];
        const float df = second.forward - first.forward;
        const float dl = second.lateral - first.lateral;
        const float norm = std::hypot(df, dl);
        if (!(norm > 0.0F)) continue;
        const float af = df / norm;
        const float al = dl / norm;
        const float bf = -al;
        const float bl = af;
        float min_a = std::numeric_limits<float>::max(), max_a = -min_a;
        float min_b = min_a, max_b = -min_a;
        for (const Point& point : hull) {
            const float a = point.forward * af + point.lateral * al;
            const float b = point.forward * bf + point.lateral * bl;
            min_a = std::min(min_a, a); max_a = std::max(max_a, a);
            min_b = std::min(min_b, b); max_b = std::max(max_b, b);
        }
        const float area = (max_a - min_a) * (max_b - min_b);
        if (area < best_area) {
            best_area = area;
            best_min_a = min_a; best_max_a = max_a;
            best_min_b = min_b; best_max_b = max_b;
            axis_f = af; axis_l = al;
        }
    }
    float edge_a = best_max_a - best_min_a;
    float edge_b = best_max_b - best_min_b;
    const float box_axis_a_f = axis_f;
    const float box_axis_a_l = axis_l;
    const float box_axis_b_f = -axis_l;
    const float box_axis_b_l = axis_f;
    const float center_a = (best_min_a + best_max_a) * 0.5F;
    const float center_b = (best_min_b + best_max_b) * 0.5F;
    const float center_forward = center_a * axis_f - center_b * axis_l;
    const float center_lateral = center_a * axis_l + center_b * axis_f;
    if (edge_b > edge_a) {
        std::swap(edge_a, edge_b);
        const float previous_axis_f = axis_f;
        axis_f = -axis_l;
        axis_l = previous_axis_f;
    }
    const float component_area = indexes.size() * forward_step * lateral_step;
    const float rectangle_area = edge_a * edge_b;
    const float rectangularity = rectangle_area > 0.0F
        ? std::clamp(PolygonArea(hull) / rectangle_area, 0.0F, 1.0F) : 0.0F;
    const float fill = rectangle_area > 0.0F
        ? std::clamp(component_area / rectangle_area, 0.0F, 1.0F) : 0.0F;
    const float orientation_error = std::atan2(std::fabs(axis_f), std::fabs(axis_l));
    if (std::fabs(edge_a - params.expected_long_edge_m) > params.long_edge_tolerance_m ||
        std::fabs(edge_b - params.expected_short_edge_m) > params.short_edge_tolerance_m ||
        orientation_error > params.max_long_edge_to_lateral_rad ||
        rectangularity < params.min_rectangularity || fill < params.min_red_fill_ratio) return out;
    const float size_score = 0.5F *
        (UnitScore(std::fabs(edge_a - params.expected_long_edge_m), params.long_edge_tolerance_m) +
         UnitScore(std::fabs(edge_b - params.expected_short_edge_m), params.short_edge_tolerance_m));
    const float orientation_score = UnitScore(orientation_error, params.max_long_edge_to_lateral_rad);
    const float weight = static_cast<float>(params.score_size_weight +
        params.score_rectangularity_weight + params.score_red_fill_weight +
        params.score_orientation_weight);
    out.valid = true;
    out.corners = {{
        {best_min_a * box_axis_a_f + best_min_b * box_axis_b_f,
         best_min_a * box_axis_a_l + best_min_b * box_axis_b_l},
        {best_max_a * box_axis_a_f + best_min_b * box_axis_b_f,
         best_max_a * box_axis_a_l + best_min_b * box_axis_b_l},
        {best_max_a * box_axis_a_f + best_max_b * box_axis_b_f,
         best_max_a * box_axis_a_l + best_max_b * box_axis_b_l},
        {best_min_a * box_axis_a_f + best_max_b * box_axis_b_f,
         best_min_a * box_axis_a_l + best_max_b * box_axis_b_l}}};
    out.center = port::BEVPoint{center_forward, center_lateral};
    out.long_edge_m = edge_a;
    out.short_edge_m = edge_b;
    out.long_axis_forward = axis_f;
    out.long_axis_lateral = axis_l;
    out.long_edge_to_lateral_rad = orientation_error;
    out.rectangularity = rectangularity;
    out.red_fill_ratio = fill;
    out.component_cells = static_cast<int>(indexes.size());
    out.quality = (static_cast<float>(params.score_size_weight) * size_score +
                   static_cast<float>(params.score_rectangularity_weight) * rectangularity +
                   static_cast<float>(params.score_red_fill_weight) * fill +
                   static_cast<float>(params.score_orientation_weight) * orientation_score) / weight;
    return out;
}

}  // namespace

port::MlOrientedRectangle DetectRedRectangle(const port::CameraPixelFrameView& frame,
                                              const BEVProjector& projector,
                                              const port::MlRoiParameters& params) {
    port::MlOrientedRectangle best{};
    best.frame_id = frame.frame_id;
    best.capture_time_ms = frame.capture_time_ms;
    if (!frame.Valid() || frame.format != port::CameraFrameFormat::kYuyv ||
        !projector.Valid() || params.grid_forward_step_m <= 0.0 ||
        params.grid_lateral_step_m <= 0.0 ||
        params.search_forward_max_m <= params.search_forward_min_m ||
        params.search_lateral_limit_m <= 0.0) return best;
    const int rows = static_cast<int>(std::floor(
        (params.search_forward_max_m - params.search_forward_min_m) /
        params.grid_forward_step_m)) + 1;
    const int cols = static_cast<int>(std::floor(
        (2.0 * params.search_lateral_limit_m) / params.grid_lateral_step_m)) + 1;
    if (rows <= 0 || cols <= 0) return best;
    std::vector<Cell> grid(static_cast<std::size_t>(rows * cols));
    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            Cell& cell = grid[static_cast<std::size_t>(row * cols + col)];
            cell.forward = static_cast<float>(
                params.search_forward_min_m + row * params.grid_forward_step_m);
            cell.lateral = static_cast<float>(
                -params.search_lateral_limit_m + col * params.grid_lateral_step_m);
            port::ImagePoint image{};
            CameraColorSample color{};
            cell.red = projector.ProjectVehicleToImage({cell.forward, cell.lateral}, image) &&
                       SampleColorAt(frame, image.row_px, image.col_px, color) &&
                       InRange(color.y, params.red_y_min, params.red_y_max) &&
                       InRange(color.u, params.red_u_min, params.red_u_max) &&
                       InRange(color.v, params.red_v_min, params.red_v_max);
        }
    }
    const int dr[4] = {-1, 0, 0, 1};
    const int dc[4] = {0, -1, 1, 0};
    for (int seed = 0; seed < rows * cols; ++seed) {
        if (!grid[static_cast<std::size_t>(seed)].red || grid[static_cast<std::size_t>(seed)].visited) continue;
        std::vector<int> queue{seed};
        grid[static_cast<std::size_t>(seed)].visited = true;
        for (std::size_t head = 0; head < queue.size(); ++head) {
            const int current = queue[head];
            const int row = current / cols, col = current % cols;
            for (int k = 0; k < 4; ++k) {
                const int nr = row + dr[k], nc = col + dc[k];
                if (nr < 0 || nr >= rows || nc < 0 || nc >= cols) continue;
                const int next = nr * cols + nc;
                Cell& cell = grid[static_cast<std::size_t>(next)];
                if (cell.red && !cell.visited) { cell.visited = true; queue.push_back(next); }
            }
        }
        port::MlOrientedRectangle candidate = MeasureComponent(
            queue,
            grid,
            static_cast<float>(params.grid_forward_step_m),
            static_cast<float>(params.grid_lateral_step_m),
            params);
        candidate.frame_id = frame.frame_id;
        candidate.capture_time_ms = frame.capture_time_ms;
        constexpr float kScoreTieEpsilon = 1.0e-6F;
        const bool better_quality =
            !best.valid || candidate.quality > best.quality + kScoreTieEpsilon;
        const bool tied_but_nearer =
            best.valid && std::fabs(candidate.quality - best.quality) <= kScoreTieEpsilon &&
            candidate.center.forward_m < best.center.forward_m;
        if (candidate.valid && (better_quality || tied_but_nearer)) best = candidate;
    }
    return best;
}

}  // namespace ls2k::vision::ml
