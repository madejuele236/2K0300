#include "presentation/internal/presentation_pages.hpp"
#include "presentation/internal/presentation_state.hpp"
#include "presentation/internal/page_common.hpp"
#include "port/presentation_ports.hpp"
#include "port/vision_observation.hpp"

#include <cstdio>

namespace primer::presentation {
namespace {

void RenderPage1DetectionStatus()
{
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
}

void RenderPage1BoundaryStatus()
{
    sprintf(txt,"L_h_guai:%d",primer::port::vision::ObserveLeftHighCorner().flag);
    primer::port::presentation::OperatorDisplay().show_string(0,100,txt);
    sprintf(txt,"R_h_guai:%d",primer::port::vision::ObserveRightHighCorner().flag);
    primer::port::presentation::OperatorDisplay().show_string(110,100,txt);
    sprintf(txt,"L_l_guai:%d",primer::port::vision::ObserveLeftLowCorner().flag);
    primer::port::presentation::OperatorDisplay().show_string(0,120,txt);
    sprintf(txt,"R_l_guai:%d",primer::port::vision::ObserveRightLowCorner().flag);
    primer::port::presentation::OperatorDisplay().show_string(110,120,txt);
    sprintf(txt,"m_l_colume:%.d ",primer::port::vision::ObserveMaxlongColumn());
    primer::port::presentation::OperatorDisplay().show_string(0,140,txt);
    sprintf(txt,"long_max:%.d",primer::port::vision::ObserveLongMax());
    primer::port::presentation::OperatorDisplay().show_string(120,140,txt);
    sprintf(txt,"distance:%.1f",primer::port::vision::ObserveDistance());
    primer::port::presentation::OperatorDisplay().show_string(0,160,txt);
    sprintf(txt,"Dir_err:%.1f",primer::port::vision::ObserveDirectionError());
    primer::port::presentation::OperatorDisplay().show_string(120,160,txt);
}

void RenderPage1TrackingStatus()
{
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
}

void RenderPage1CornerCoordinates()
{
    sprintf(txt,"Rh.row:%d ",primer::port::vision::ObserveRightHighCorner().row);
    primer::port::presentation::OperatorDisplay().show_string(110,260,txt);
    sprintf(txt,"Lh.row:%d ",primer::port::vision::ObserveLeftHighCorner().row);
    primer::port::presentation::OperatorDisplay().show_string(0,260,txt);
    sprintf(txt,"Rh.column:%d ",primer::port::vision::ObserveRightHighCorner().column);
    primer::port::presentation::OperatorDisplay().show_string(110,280,txt);
    sprintf(txt,"Lh.column:%d ",primer::port::vision::ObserveLeftHighCorner().column);
    primer::port::presentation::OperatorDisplay().show_string(0,280,txt);
}

}  // namespace

void RenderPage1(void)
{
    if(oled_flag == 1)
    {
        internal::RenderTrackedBinaryImage();
        RenderPage1DetectionStatus();
        RenderPage1BoundaryStatus();
        RenderPage1TrackingStatus();
        RenderPage1CornerCoordinates();
        internal::RenderPageFooter();
    }
}

}  // namespace primer::presentation
