#include "presentation/internal/presentation_pages.hpp"
#include "presentation/internal/presentation_state.hpp"
#include "port/presentation_ports.hpp"
#include "port/vision_observation.hpp"

#include <cstdio>
#include <cstring>
#include <opencv2/core.hpp>

namespace primer::presentation {

void RenderRedDebugPage(void)
{
    //红色阈值调试界面
if(oled_flag == 4)
{
    if(!primer::port::vision::ObserveResizedFrame().empty())
    {
        cv::Mat debug_hsv, debug_mask1, debug_mask2, debug_mask;
        debug_mask.create(primer::port::vision::ObserveResizedFrame().size(), CV_8UC1);
        debug_mask.setTo(0); // 初始化为黑
        if(debug_mask.empty())
        {
            printf("debug_resized is empty\n");
            return;
        }

        for(int y = 0; y < primer::port::vision::ObserveResizedFrame().rows; y++)
        {
            const cv::Vec3b* ptr = primer::port::vision::ObserveResizedFrame().ptr<cv::Vec3b>(y);
            uchar* mask_ptr = debug_mask.ptr<uchar>(y);

            for(int x = 0; x < primer::port::vision::ObserveResizedFrame().cols; x++)
            {
                int b = ptr[x][0];
                int g = ptr[x][1];
                int r = ptr[x][2];

                // 这里必须与 DetectRedBlock 中的逻辑完全一致
                if (r > primer::port::presentation::ObserveParameters().debug_rgb_r_min && (r - g) > primer::port::presentation::ObserveParameters().debug_rgb_rg_diff && (r - b) > primer::port::presentation::ObserveParameters().debug_rgb_rb_diff)
                {
                    mask_ptr[x] = 255; // 白色表示检测到红色
                }
            }
        }
            for(int i = 0; i < primer::port::vision::kProcessedHeight; i++)
            {
                // 直接拷贝一行数据，效率比逐像素遍历高
                memcpy(Image_IFS[i], debug_mask.ptr<uint8_t>(i), primer::port::vision::kProcessedWidth);
            }

            primer::port::presentation::OperatorDisplay().show_gray_image(0, 0, (const uint8_t*)Image_IFS, primer::port::vision::kProcessedWidth, primer::port::vision::kProcessedHeight, primer::port::vision::kProcessedWidth, primer::port::vision::kProcessedHeight, 1);
    }
    else{
        printf("resizedFrame is empty\n");
        return;
    }
    sprintf(txt, "R_Min : %d  ", primer::port::presentation::ObserveParameters().debug_rgb_r_min);
    primer::port::presentation::OperatorDisplay().show_string(0, 60, txt);

    sprintf(txt, "R-G Diff : %d  ", primer::port::presentation::ObserveParameters().debug_rgb_rg_diff);
    primer::port::presentation::OperatorDisplay().show_string(0, 80, txt);

    sprintf(txt, "R-B Diff: %d ", primer::port::presentation::ObserveParameters().debug_rgb_rb_diff);
    primer::port::presentation::OperatorDisplay().show_string(0, 100, txt);


    sprintf(txt,"L_h_guai:%d",primer::port::vision::ObserveLeftHighCorner().flag);
    primer::port::presentation::OperatorDisplay().show_string(0,140,txt);

    sprintf(txt,"R_h_guai:%d",primer::port::vision::ObserveRightHighCorner().flag);
    primer::port::presentation::OperatorDisplay().show_string(120,140,txt);

    sprintf(txt,"red_find_x:%d",primer::port::vision::ObserveRedFindX());
    primer::port::presentation::OperatorDisplay().show_string(0,160,txt);

    sprintf(txt,"Flag.picture:%d",primer::port::vision::ObserveElementFacts().picture);
    primer::port::presentation::OperatorDisplay().show_string(120,160,txt);

    sprintf(txt,"red_find_y:%d",primer::port::vision::ObserveRedFindY());
    primer::port::presentation::OperatorDisplay().show_string(0,180,txt);

    sprintf(txt,"Parameter:%d",Parameter_flag);
    primer::port::presentation::OperatorDisplay().show_string(0,300,txt);

    sprintf(txt,"oled_page:%d",oled_flag);
    primer::port::presentation::OperatorDisplay().show_string(130,300,txt);
}
}

}  // namespace primer::presentation
