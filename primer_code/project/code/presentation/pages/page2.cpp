#include "presentation/internal/presentation_pages.hpp"
#include "presentation/internal/presentation_state.hpp"
#include "port/presentation_ports.hpp"
#include "port/vision_observation.hpp"

#include <cstdio>

namespace primer::presentation {

void RenderPage2(void)
{
if(oled_flag == 2)
{
    for (int i = 0; i <=primer::port::vision::kProcessedHeight; i++)
     {
         for (int j = 0; j < primer::port::vision::kProcessedWidth; j++) {if (primer::port::vision::ObserveBinaryImage()[i][j] > 0)Image_IFS[i][j] = 255;else Image_IFS[i][j] = 0;}
     }
     for (int j = 0; j < primer::port::vision::kProcessedHeight; j++) {Image_IFS[j][primer::port::vision::ObserveLeftSideline()[j]] = 0;}
     for (int j = 0; j < primer::port::vision::kProcessedHeight; j++) {Image_IFS[j][primer::port::vision::ObserveRightSideline()[j]] = 0;}
     for (int j = 0; j < primer::port::vision::kProcessedHeight; j++) {Image_IFS[j][primer::port::vision::ObserveMidline()[j]] = 0;}
       primer::port::presentation::OperatorDisplay().show_gray_image(0,0,(const uint8_t*)Image_IFS,primer::port::vision::kProcessedWidth,primer::port::vision::kProcessedHeight,primer::port::vision::kProcessedWidth,primer::port::vision::kProcessedHeight,1);


        sprintf(txt,"Dir_err:%f  ",primer::port::vision::ObserveDirectionError());
        primer::port::presentation::OperatorDisplay().show_string(0,60,txt);

         sprintf(txt,"Flag.Zhangai:%d",primer::port::vision::ObserveElementFacts().Zhangai);
         primer::port::presentation::OperatorDisplay().show_string(0,80,txt);

        sprintf(txt,"Flag.Redblock:%d:",primer::port::vision::ObserveElementFacts().Redblock);
        primer::port::presentation::OperatorDisplay().show_string(0,100,txt);

         sprintf(txt,"Flag.picture:%d:",primer::port::vision::ObserveElementFacts().picture);
         primer::port::presentation::OperatorDisplay().show_string(0,120,txt);

         sprintf(txt,"encode_l_total:%d     ",primer::port::presentation::ObserveTelemetry().left_encoder_total);
         primer::port::presentation::OperatorDisplay().show_string(0,140,txt);

         sprintf(txt,"encode_r_total:%d     ",primer::port::presentation::ObserveTelemetry().right_encoder_total);
         primer::port::presentation::OperatorDisplay().show_string(0,160,txt);




        // sprintf(txt,"Image.column:%d  ",imgInfo.max_column);
        // ips200.show_string(0,160,txt);

        sprintf(txt,"encoder_abs:%d   ",primer::port::presentation::ObserveTelemetry().encoder_distance);
        primer::port::presentation::OperatorDisplay().show_string(0,180,txt);

        sprintf(txt,"jump_point:%d  ",primer::port::vision::ObserveJumpPoint());
        primer::port::presentation::OperatorDisplay().show_string(0,200,txt);

        sprintf(txt,"icm_data.gyro_z:%.2f  ",primer::port::presentation::ObserveImu().gyro_z );
        primer::port::presentation::OperatorDisplay().show_string(0,220,txt);
        sprintf(txt,"icm_data.gyro_y:%.2f  ",primer::port::presentation::ObserveImu().gyro_y );
        primer::port::presentation::OperatorDisplay().show_string(0,240,txt);
        sprintf(txt,"Flag.Zebra_cross:%d ",primer::port::vision::ObserveElementFacts().Zebra_cross);
        primer::port::presentation::OperatorDisplay().show_string(0,260,txt);
        sprintf(txt,"dis:%.2f",primer::port::vision::ObserveDistance());
        primer::port::presentation::OperatorDisplay().show_string(150,260,txt);

        sprintf(txt,"Parameter:%d",Parameter_flag);
        primer::port::presentation::OperatorDisplay().show_string(0,300,txt);

        sprintf(txt,"oled_page:%d",oled_flag);
        primer::port::presentation::OperatorDisplay().show_string(130,300,txt);
}
}

}  // namespace primer::presentation
