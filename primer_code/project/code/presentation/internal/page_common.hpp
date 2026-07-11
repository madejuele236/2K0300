#pragma once

#include "presentation/internal/presentation_state.hpp"
#include "port/presentation_ports.hpp"
#include "port/vision_observation.hpp"

#include <cstdio>

namespace primer::presentation::internal {

inline void RenderTrackedBinaryImage()
{
    for (int i = 0; i <= primer::port::vision::kProcessedHeight; i++)
    {
        for (int j = 0; j < primer::port::vision::kProcessedWidth; j++)
        {
            if (primer::port::vision::ObserveBinaryImage()[i][j] > 0) Image_IFS[i][j] = 255;
            else Image_IFS[i][j] = 0;
        }
    }
    for (int j = 0; j < primer::port::vision::kProcessedHeight; j++)
    {
        Image_IFS[j][primer::port::vision::ObserveLeftSideline()[j]] = 0;
        Image_IFS[j][primer::port::vision::ObserveLeftSideline()[j + 1]] = 0;
    }
    for (int j = 0; j < primer::port::vision::kProcessedHeight; j++)
    {
        Image_IFS[j][primer::port::vision::ObserveRightSideline()[j]] = 0;
        Image_IFS[j][primer::port::vision::ObserveRightSideline()[j - 1]] = 0;
    }
    for (int j = 0; j < primer::port::vision::kProcessedHeight; j++)
    {
        Image_IFS[j][primer::port::vision::ObserveMidline()[j]] = 0;
    }
    primer::port::presentation::OperatorDisplay().show_gray_image(
        0, 0, (const uint8_t*)Image_IFS,
        primer::port::vision::kProcessedWidth, primer::port::vision::kProcessedHeight,
        primer::port::vision::kProcessedWidth, primer::port::vision::kProcessedHeight, 1);
}

inline void RenderPageFooter()
{
    sprintf(txt, "Parameter:%d", Parameter_flag);
    primer::port::presentation::OperatorDisplay().show_string(0, 300, txt);
    sprintf(txt, "oled_page:%d", oled_flag);
    primer::port::presentation::OperatorDisplay().show_string(130, 300, txt);
}

}  // namespace primer::presentation::internal
