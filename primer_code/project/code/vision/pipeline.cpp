#include <opencv2/imgproc.hpp>
#include "internal/dependencies/pipeline_dependencies.hpp"
/***************************************************图像处理******************************************************/
void ImageDeal()
{
    lq_frame = cam.get_frame_raw();
    if(lq_frame.empty()){
        return;
    }
      cv::resize(lq_frame,resizedFrame,cv::Size(LCDW_1,LCDH_1),0,0,cv::INTER_AREA);

    // RedBlockProcess(resizedFrame);
    // camera_server.update_frame_mat(lq_frame);//打开图传服务器

      cv::cvtColor(resizedFrame,grayFrame,cv::COLOR_BGR2GRAY);

    //   cv::warpAffine(grayFrame, translatedFrame, translationMatrix, grayFrame.size(),
    //                  cv::INTER_LINEAR, cv::BORDER_CONSTANT, 0);

//   //  将OpenCV图像数据复制到图像数组 采用memcpy函数加快处理速度
    for (int i = 0; i < LCDH_1; i++)
    {
        uint8_t *p = grayFrame.ptr<uint8_t>(i);
        for(int w = 0; w < LCDW_1; w++)
        {
            Image_Use[i][w] = p[w];

        }
    }
         Get01change_dajin();

        // my_sobel(Image_Zip,Image_Use); //压缩后的图像数组,

        // my_sobel_dajin(Image_Zip,Image_Use);

        imgInfoInit();

        Get_ImageTop();

        Draw_BlackSideline(Image_Use);//画边线

        Find_Sideline(imgInfo.bottom-1,imgInfo.top+ 1);//找边线

        if(Flag.Huandao_L>0||Flag.Huandao_R>0)
        Find_Guaidian();  //找拐点
        else
        Find_Guaidian1();  //找拐点

        straight_judge();

        zebra_corssing();

        if(Flag.Zebra_cross==3)
        {
        ramp();
        }

        picture();

        if(Flag.Huandao_L==0&&Flag.Huandao_R==0)
        small_rock();
        // }

        // if(Flag.Huandao_L!=1&&Flag.Huandao_R!=1)


       Huandao_R_imu();
       Huandao_L_imu();


       if(Flag.Huandao_L!=1&&Flag.Huandao_R!=1&&Flag.Huandao_L!=2&&Flag.Huandao_R!=2&&Flag.Huandao_L!=3&&Flag.Huandao_R!=3
       &&Flag.Huandao_L!=4&&Flag.Huandao_R!=4&&Flag.Huandao_L!=5&&Flag.Huandao_R!=5&&Flag.Huandao_L!=6&&Flag.Huandao_R!=6)
        Buxian();

        Find_Midline();

     //   dynamic_forward();



        Err_Sum();




        protect();
}
