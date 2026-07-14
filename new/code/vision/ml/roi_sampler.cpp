#include "vision/ml/roi_sampler.hpp"

#include <cmath>

#include "vision/image/color_sampler.hpp"

namespace ls2k::vision::ml {
namespace {

port::MlGrayRoi32 InvalidRoi(const port::CameraPixelFrameView& frame,
                             const char* reason) {
    port::MlGrayRoi32 out{};
    out.frame_id = frame.frame_id;
    out.reason = reason;
    return out;
}

}  // namespace

port::MlGrayRoi32 SampleSquareRoi32(const port::CameraPixelFrameView& frame,
                                    const BEVProjector& projector,
                                    const port::MlOrientedRectangle& rectangle,
                                    const port::MlRoiParameters& params) {
    port::MlGrayRoi32 out{};
    if (!frame.Valid() || frame.format != port::CameraFrameFormat::kYuyv ||
        !projector.Valid() || !rectangle.valid ||
        !(params.expected_long_edge_m > 0.0) ||
        !(params.expected_short_edge_m > 0.0)) {
        return InvalidRoi(frame, "invalid_input");
    }
    const float axis_norm = std::hypot(rectangle.long_axis_forward, rectangle.long_axis_lateral);
    if (!(axis_norm > 0.0F)) return InvalidRoi(frame, "invalid_rectangle_axis");
    float lf = rectangle.long_axis_forward / axis_norm;
    float ll = rectangle.long_axis_lateral / axis_norm;
    // Canonicalize columns from negative to positive vehicle lateral so the
    // classifier never receives an arbitrary left/right mirror.
    if (ll < 0.0F || (ll == 0.0F && lf < 0.0F)) {
        lf = -lf;
        ll = -ll;
    }
    // Of the two normals to the observed long edge, choose the one with a
    // positive vehicle-forward component. The square starts at the rectangle's
    // forward long edge and extends away from it in this direction.
    float nf = -ll;
    float nl = lf;
    if (nf < 0.0F) {
        nf = -nf;
        nl = -nl;
    }
    const float edge_center_forward =
        rectangle.center.forward_m +
        (0.5F * static_cast<float>(params.expected_short_edge_m) +
         static_cast<float>(params.crop_forward_offset_m)) * nf +
        static_cast<float>(params.crop_long_offset_m) * lf;
    const float edge_center_lateral =
        rectangle.center.lateral_m +
        (0.5F * static_cast<float>(params.expected_short_edge_m) +
         static_cast<float>(params.crop_forward_offset_m)) * nl +
        static_cast<float>(params.crop_long_offset_m) * ll;
    const float square_edge_m = static_cast<float>(params.expected_long_edge_m);
    for (int row = 0; row < port::kMlRoiSide; ++row) {
        // Preserve normal raster orientation: the top output row is the
        // farthest point along the vehicle-forward normal, while the bottom
        // row is nearest the marker's forward edge.
        const float along_forward =
            ((port::kMlRoiSide - row - 0.5F) / port::kMlRoiSide) *
            square_edge_m;
        for (int col = 0; col < port::kMlRoiSide; ++col) {
            const float along_long =
                ((col + 0.5F) / port::kMlRoiSide - 0.5F) * square_edge_m;
            const port::BEVPoint point{
                edge_center_forward + along_long * lf + along_forward * nf,
                edge_center_lateral + along_long * ll + along_forward * nl};
            port::ImagePoint image{};
            std::uint8_t gray = 0;
            if (!projector.ProjectVehicleToImage(point, image) ||
                !SampleRgbMeanAt(frame, image.row_px, image.col_px, gray)) {
                return InvalidRoi(frame, "source_sample_unavailable");
            }
            out.gray[static_cast<std::size_t>(row * port::kMlRoiSide + col)] = gray;
        }
    }
    out.valid = true;
    out.frame_id = frame.frame_id;
    out.long_axis_forward = lf;
    out.long_axis_lateral = ll;
    out.forward_normal_forward = nf;
    out.forward_normal_lateral = nl;
    out.reason = "ok";
    return out;
}

}  // namespace ls2k::vision::ml
