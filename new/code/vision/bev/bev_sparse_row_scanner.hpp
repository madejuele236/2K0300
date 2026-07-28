#ifndef LS2K_VISION_BEV_SPARSE_ROW_SCANNER_HPP
#define LS2K_VISION_BEV_SPARSE_ROW_SCANNER_HPP

#include <vector>

#include "port/camera_frame_types.hpp"
#include "port/binary_model_types.hpp"
#include "port/runtime_parameter_types.hpp"
#include "vision/bev/bev_row_facts.hpp"
#include "vision/bev/bev_sample_projection_lut.hpp"

namespace ls2k::vision {

std::vector<BEVSimpleRowScan> ScanSparseRows(const port::CameraPixelFrameView& frame,
                                             const port::BinaryModelState& binary_model,
                                             const port::RuntimeParameters& params,
                                             const BEVSampleProjectionLut& lut);

}  // namespace ls2k::vision

#endif  // LS2K_VISION_BEV_SPARSE_ROW_SCANNER_HPP
