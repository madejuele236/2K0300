#include "presentation/internal/presentation_pages.hpp"
#include "presentation/internal/presentation_state.hpp"
#include "port/presentation_ports.hpp"
#include "port/vision_observation.hpp"

#include <cstdio>

namespace primer::presentation {

void RenderPage1(void)
{
if(oled_flag ==1){
    for (int i = 0; i <=primer::port::vision::kProcessedHeight; i++)
     {
         for (int j = 0; j < primer::port::vision::kProcessedWidth; j++) {if (primer::port::vision::ObserveBinaryImage()[i][j] > 0)Image_IFS[i][j] = 255;else Image_IFS[i][j] = 0;}
     }
     for (int j = 0; j < primer::port::vision::kProcessedHeight; j++) {Image_IFS[j][primer::port::vision::ObserveLeftSideline()[j]] = 0;Image_IFS[j][primer::port::vision::ObserveLeftSideline()[j+1]] = 0;}
     for (int j = 0; j < primer::port::vision::kProcessedHeight; j++) {Image_IFS[j][primer::port::vision::ObserveRightSideline()[j]] = 0;Image_IFS[j][primer::port::vision::ObserveRightSideline()[j-1]] = 0;}
     for (int j = 0; j < primer::port::vision::kProcessedHeight; j++) {Image_IFS[j][primer::port::vision::ObserveMidline()[j]] = 0;}
       primer::port::presentation::OperatorDisplay().show_gray_image(0,0,(const uint8_t*)Image_IFS,primer::port::vision::kProcessedWidth,primer::port::vision::kProcessedHeight,primer::port::vision::kProcessedWidth,primer::port::vision::kProcessedHeight,1);

                //    sprintf(txt,"yaw:%.1f ",icm_data.yaw);
                //    ips200.show_string(100,0,txt);


                //    sprintf(txt,"err:%.1f ",Yaw_Huandao_err);
                //    ips200.show_string(100,20,txt);

                //    sprintf(txt,"dl1x:%d ",dl1x_distance_raw);
                //    ips200.show_string(100,40,txt);

                //    sprintf(txt,"jump_point:%d ",jump_point);
                //    ips200.show_string(100,60,txt);

                //    sprintf(txt,"Huandao_L:%d",Flag.Huandao_L);
                //    ips200.show_string(0,80,txt);

                //    sprintf(txt,"Huandao_R:%d",Flag.Huandao_R);
                //    ips200.show_string(110,80,txt);




                //    sprintf(txt,"Huandao_L:%d",Flag.Huandao_L);
                //    ips200.show_string(0,80,txt);

                //    sprintf(txt,"Huandao_R:%d",Flag.Huandao_R);
                //    ips200.show_string(110,80,txt);
// //2
//                    sprintf(txt,"maxlong_colume:%.d ",maxlong_colume);
//                    ips200.show_string(100,0,txt);


//                    sprintf(txt,"long_max:%.d",long_max);
//                    ips200.show_string(100,20,txt);

//                    sprintf(txt,"dl1x:%d ",dl1x_distance_raw);
//                    ips200.show_string(100,40,txt);

//                    sprintf(txt,"jump_point:%d ",jump_point);
//                    ips200.show_string(100,60,txt);

//2


                   sprintf(txt,"pic_white:%d  ",primer::port::vision::ObservePictureWhite());
                   primer::port::presentation::OperatorDisplay().show_string(100,0,txt);


                   sprintf(txt,"pic_black:%d  ",primer::port::vision::ObservePictureBlack());
                   primer::port::presentation::OperatorDisplay().show_string(100,20,txt);

                   sprintf(txt,"point1:%d  ",primer::port::vision::ObserveJumpPointSecondary());
                   primer::port::presentation::OperatorDisplay().show_string(100,40,txt);

                   sprintf(txt,"black_ratio:%.3f",primer::port::vision::ObserveBlackRatio());
                   primer::port::presentation::OperatorDisplay().show_string(100,60,txt);


                   sprintf(txt,"Huandao_L:%d",primer::port::vision::ObserveElementFacts().Huandao_L);
                   primer::port::presentation::OperatorDisplay().show_string(0,80,txt);

                   sprintf(txt,"Huandao_R:%d",primer::port::vision::ObserveElementFacts().Huandao_R);
                   primer::port::presentation::OperatorDisplay().show_string(110,80,txt);




              sprintf(txt,"L_h_guai:%d",primer::port::vision::ObserveLeftHighCorner().flag);
              primer::port::presentation::OperatorDisplay().show_string(0,100,txt);

              sprintf(txt,"R_h_guai:%d",primer::port::vision::ObserveRightHighCorner().flag);
              primer::port::presentation::OperatorDisplay().show_string(110,100,txt);

              sprintf(txt,"L_l_guai:%d",primer::port::vision::ObserveLeftLowCorner().flag);
              primer::port::presentation::OperatorDisplay().show_string(0,120,txt);

              sprintf(txt,"R_l_guai:%d",primer::port::vision::ObserveRightLowCorner().flag);
              primer::port::presentation::OperatorDisplay().show_string(110,120,txt);


//2
                   sprintf(txt,"m_l_colume:%.d ",primer::port::vision::ObserveMaxlongColumn());
                   primer::port::presentation::OperatorDisplay().show_string(0,140,txt);


                   sprintf(txt,"long_max:%.d",primer::port::vision::ObserveLongMax());
                   primer::port::presentation::OperatorDisplay().show_string(120,140,txt);




            //   sprintf(txt,"L_lose:%d  ",imgInfo.L_loselineSum);
            //   ips200.show_string(0,140,txt);

            //   sprintf(txt,"R_lose:%d  ",imgInfo.R_loselineSum);
            //   ips200.show_string(80,140,txt);

              sprintf(txt,"distance:%.1f",primer::port::vision::ObserveDistance());
              primer::port::presentation::OperatorDisplay().show_string(0,160,txt);

              sprintf(txt,"Dir_err:%.1f",primer::port::vision::ObserveDirectionError());
              primer::port::presentation::OperatorDisplay().show_string(120,160,txt);

                    //   sprintf(txt,"L_strai:%d",imgInfo.L_straight_flag);
                    //   ips200.show_string(0,180,txt);

                    //   sprintf(txt,"R_strai:%d",imgInfo.R_straight_flag);
                    //   ips200.show_string(120,180,txt);

                   sprintf(txt,"R_row+10:%d     ",primer::port::presentation::DistanceToRow(primer::port::vision::ObserveRowDistance()[primer::port::vision::ObserveRightHighCorner().row]+10));
                   primer::port::presentation::OperatorDisplay().show_string(0,180,txt);



                   sprintf(txt,"Picture:%d",primer::port::vision::ObserveElementFacts().picture);
                   primer::port::presentation::OperatorDisplay().show_string(110,180,txt);


                      sprintf(txt,"Both_l:%d ",primer::port::vision::ObserveImageInformation().Both_lose);
                      primer::port::presentation::OperatorDisplay().show_string(0,200,txt);

                      sprintf(txt,"top:%d ",primer::port::vision::ObserveImageInformation().top);
                      primer::port::presentation::OperatorDisplay().show_string(110,200,txt);

                      sprintf(txt,"Rh.row1:%d ",primer::port::vision::ObserveRightHighCornerSecondary().row);
                      primer::port::presentation::OperatorDisplay().show_string(110,220,txt);

                      sprintf(txt,"Lh.row1:%d ",primer::port::vision::ObserveLeftHighCornerSecondary().row);
                      primer::port::presentation::OperatorDisplay().show_string(0,220,txt);


                      sprintf(txt,"Rh.column1:%d ",primer::port::vision::ObserveRightHighCornerSecondary().column);
                      primer::port::presentation::OperatorDisplay().show_string(110,240,txt);

                      sprintf(txt,"Lh.column1:%d ",primer::port::vision::ObserveLeftHighCornerSecondary().column);
                      primer::port::presentation::OperatorDisplay().show_string(0,240,txt);




                      sprintf(txt,"Rh.row:%d ",primer::port::vision::ObserveRightHighCorner().row);
                      primer::port::presentation::OperatorDisplay().show_string(110,260,txt);

                      sprintf(txt,"Lh.row:%d ",primer::port::vision::ObserveLeftHighCorner().row);
                      primer::port::presentation::OperatorDisplay().show_string(0,260,txt);


                      sprintf(txt,"Rh.column:%d ",primer::port::vision::ObserveRightHighCorner().column);
                      primer::port::presentation::OperatorDisplay().show_string(110,280,txt);

                      sprintf(txt,"Lh.column:%d ",primer::port::vision::ObserveLeftHighCorner().column);
                      primer::port::presentation::OperatorDisplay().show_string(0,280,txt);


                    //     sprintf(txt,"picture_num1:%d ",picture_second_num);
                    //   ips200.show_string(110,280,txt);

                    //   sprintf(txt,"picture_num2:%d ",picture_first_num);
                    //   ips200.show_string(0,280,txt);


                     sprintf(txt,"Parameter:%d",Parameter_flag);
                     primer::port::presentation::OperatorDisplay().show_string(0,300,txt);

                     sprintf(txt,"oled_page:%d",oled_flag);
                     primer::port::presentation::OperatorDisplay().show_string(130,300,txt);
}
}

}  // namespace primer::presentation
