#include <opencv2/imgproc.hpp>
#include "internal/dependencies/pipeline_dependencies.hpp"

namespace {

static inline void ResizeAndConvertFrame()
{
    cv::resize(lq_frame,resizedFrame,cv::Size(LCDW_1,LCDH_1),0,0,cv::INTER_AREA);
    cv::cvtColor(resizedFrame,grayFrame,cv::COLOR_BGR2GRAY);
}

static inline void CopyGrayFrameToImageUse()
{
    for (int i = 0; i < LCDH_1; i++)
    {
        uint8_t *p = grayFrame.ptr<uint8_t>(i);
        for(int w = 0; w < LCDW_1; w++)
        {
            Image_Use[i][w] = p[w];

        }
    }
}

static inline void BinarizeFrame()
{
    Get01change_dajin();
}

static inline void ExtractTrackFacts()
{
    imgInfoInit();
    Get_ImageTop();
    Draw_BlackSideline(Image_Use);
    Find_Sideline(imgInfo.bottom-1,imgInfo.top+ 1);

    if(Flag.Huandao_L>0||Flag.Huandao_R>0)
    {
        Find_Guaidian();
    }
    else
    {
        Find_Guaidian1();
    }

    straight_judge();
}

static inline void DetectScenes()
{
    zebra_corssing();
    if(Flag.Zebra_cross==3)
    {
        ramp();
    }

    picture();

    if(Flag.Huandao_L==0&&Flag.Huandao_R==0)
    {
        small_rock();
    }

    Huandao_R_imu();
    Huandao_L_imu();
}

static inline void RepairTrackLines()
{
    if(Flag.Huandao_L!=1&&Flag.Huandao_R!=1&&Flag.Huandao_L!=2&&Flag.Huandao_R!=2&&Flag.Huandao_L!=3&&Flag.Huandao_R!=3
    &&Flag.Huandao_L!=4&&Flag.Huandao_R!=4&&Flag.Huandao_L!=5&&Flag.Huandao_R!=5&&Flag.Huandao_L!=6&&Flag.Huandao_R!=6)
    {
        Buxian();
    }
}

static inline void CompleteVisionPipeline()
{
    Find_Midline();
    Err_Sum();
    protect();
}

}  // namespace

void ImageDeal()
{
    lq_frame = primer::port::VisionCamera().get_frame_raw();
    if(lq_frame.empty()){
        return;
    }

    ResizeAndConvertFrame();
    CopyGrayFrameToImageUse();
    BinarizeFrame();
    ExtractTrackFacts();
    DetectScenes();
    RepairTrackLines();
    CompleteVisionPipeline();
}
