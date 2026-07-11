#include "../internal/dependencies/corners_dependencies.hpp"

struct Guaidian L_l_guai, L_h_guai, R_l_guai, R_h_guai;  //拐点信息结构体
int Guai_row = 0;

namespace {

void ResetPrimaryCorners(void)
{
    L_l_guai.flag = 0;
    L_h_guai.flag = 0;
    R_l_guai.flag = 0;
    R_h_guai.flag = 0;


    if(L_l_guai.flag == 0)
        L_l_guai.row = LCDH_1 - 1;
    if(R_l_guai.flag == 0)
        R_l_guai.row = LCDH_1 - 1;
}

void ScanPrimaryUpperCorners(void)
{
    //找左上，右上拐点
    for(int i = imgInfo.top + 2; i <= imgInfo.bottom - 5; i++)
    {

        /**************左上拐点**************/
        if(
             Left_Sideline[i] > 3 &&
             (
                  /*    *0
                      0100
                           */
                 (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
                  && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0000
                           */
                  (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                   && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && !Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      00010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0]&& Image_Use[i + 1][Left_Sideline[i] + 1] && !Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
//                   ||/*    *0
//                          01000
//                            */
// (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
// && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 3]
// && Image_Use[i + 1][Left_Sideline[i] - 2])

             )
             && white_width[i + 2] - white_width[i - 0] > 5
             && xielv_sideline(i, Left_Sideline[i], i + 1, Left_Sideline[i + 1], 'k') < 1
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Left_Sideline[i - 1] - Left_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Left_Sideline[i], i - 3, Left_Sideline[i - 3], 'k') - xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k')) > 0.5
//             && xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k') < 1
             && L_h_guai.flag == 0
             && i < L_l_guai.row
        )
        {
            L_h_guai.row = i + 1;
            L_h_guai.column = (uint8)Left_Sideline[i];
            L_h_guai.flag = 1;

//            if(R_h_guai.flag)
//                break;

        }

        /**************右上拐点**************/
        if(
             Right_Sideline[i] < LCDW_1 - 3 &&
             (
                   /*  0*
                       0010
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && !Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0100
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && !Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                      01000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 1] && !Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
//                    ||/*    0*
//                          00010
//                     */
// (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0]
// && Image_Use[i + 1][Right_Sideline[i] + 1] && !Image_Use[i + 1][Right_Sideline[i] + 3]
// && Image_Use[i + 1][Right_Sideline[i] + 2])

             )
             && white_width[i + 2] - white_width[i - 0] > 5
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Right_Sideline[i - 1] - Right_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
             && xielv_sideline(i, Right_Sideline[i], i + 1, Right_Sideline[i + 1], 'k') > -1
             && R_h_guai.flag == 0
             && i < R_l_guai.row
           )
        {
            R_h_guai.row = i + 1;
            R_h_guai.column = (uint8)Right_Sideline[i];
            R_h_guai.flag = 1;

//            if(L_h_guai.flag)
//                break;
        }

    }
}

void ScanPrimaryLowerCorners(void)
{
        //找左下，右下拐点
        for(int i = imgInfo.bottom - 3; i >= imgInfo.top + 3 && Flag.Huandao_L != 3 && Flag.Huandao_R != 3; i--)
        {
            if(L_h_guai.flag == 0)
                L_h_guai.row = 1;
            if(R_h_guai.flag == 0)
                R_h_guai.row = 1;
            /**************左下拐点**************/
            if(
                 Left_Sideline[i] > 3 &&
                 (
                      /*  0000
                            *0
                                */
                     (Image_Use[i][Left_Sideline[i] +0] && Image_Use[i - 1][Left_Sideline[i] + 0]
                      && Image_Use[i - 1][Left_Sideline[i] - 1] && Image_Use[i - 1][Left_Sideline[i] - 2]  && Image_Use[i - 1][Left_Sideline[i] -3])
                     ||
                     /*  0100
                           *0
                               */
                     (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] - 1]
                      && !Image_Use[i - 1][Left_Sideline[i] - 2] && Image_Use[i - 1][Left_Sideline[i] - 3])
                      ||
                      /*  0010
                            *0
                                */
                      (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] + 0] && !Image_Use[i - 1][Left_Sideline[i] - 1]
                       && Image_Use[i - 1][Left_Sideline[i] - 2] && Image_Use[i - 1][Left_Sideline[i] - 3])
                 )
                 /*上面三行的白行都比这一行多*/
                 && white_width[i - 2] - white_width[i + 1] > 10
                 && white_width[i - 2] > white_width[i] && white_width[i - 3] > white_width[i] && white_width[i - 4] > white_width[i]
                 && i > L_h_guai.row
                 && L_l_guai.flag == 0
            )
            {
                L_l_guai.row = i - 1;
                L_l_guai.column = (uint8)Left_Sideline[i];
                L_l_guai.flag = 1;

//                if(R_l_guai.flag)
//                    break;

            }

            /**************右下拐点**************/
            if(
                 Right_Sideline[i] < LCDW_1 - 3 &&
                 (
                         /*
                              0000
                              0*
                          */
                         (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0]
                         && Image_Use[i - 1][Right_Sideline[i] + 1] && Image_Use[i - 1][Right_Sideline[i] + 2] && Image_Use[i - 1][Right_Sideline[i] + 3])
                         ||
                         /*
                              0010
                              0*
                          */
                         (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] + 1]
                          && !Image_Use[i - 1][Right_Sideline[i] + 2] && Image_Use[i - 1][Right_Sideline[i] + 3])
                          ||
                          /*
                               0001
                               0*
                           */
                          (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] + 1]
                           && Image_Use[i - 1][Right_Sideline[i] + 2] && !Image_Use[i - 1][Right_Sideline[i] + 3])
                 )
                 /*上面三行的白行都比这一行多*/
                 && white_width[i - 2] > white_width[i] && white_width[i - 3] > white_width[i]
                 && white_width[i - 2] - white_width[i + 1] > 10
//                 && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
//                 && xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') > 1
                 && i > R_h_guai.row
                 && R_l_guai.flag == 0
            )
            {
                R_l_guai.row = i - 1;
                R_l_guai.column = (uint8)Right_Sideline[i];
                R_l_guai.flag = 1;

//                if(L_l_guai.flag)
//                    break;
            }
        }
}

void ApplyPrimaryRoundaboutOverrides(void)
{
int l_guai_max=0,r_guai_max=93;//,l_guai_line,r_guai_line
if(Flag.Huandao_R == 4)
{
    for(int i = imgInfo.top + 2; i <54; i++)
    {
            if(Left_Sideline[i]> l_guai_max)//
            {
                l_guai_max = Left_Sideline[i];
                L_l_guai.row = i ;
            }
    }

                L_l_guai.column = (uint8)Left_Sideline[L_l_guai.row]+3;
                L_l_guai.flag = 1;
}
if(Flag.Huandao_L == 4)
{
    for(int i = imgInfo.top + 2; i <54; i++)
    {
            if(Right_Sideline[i]< r_guai_max)//
            {
                r_guai_max = Right_Sideline[i];
                R_l_guai.row = i ;
            }
    }

                R_l_guai.column = (uint8)Right_Sideline[R_l_guai.row]-3;
                R_l_guai.flag = 1;
}
}

}  // namespace

void Find_Guaidian(void)
{
    ResetPrimaryCorners();
    ScanPrimaryUpperCorners();
    ScanPrimaryLowerCorners();
    ApplyPrimaryRoundaboutOverrides();
}

namespace {

void ResetSecondaryCorners(void)
{
    L_l_guai.flag = 0;
    L_h_guai.flag = 0;
    R_l_guai.flag = 0;
    R_h_guai.flag = 0;


    if(L_l_guai.flag == 0)
        L_l_guai.row = LCDH_1 - 1;
    if(R_l_guai.flag == 0)
        R_l_guai.row = LCDH_1 - 1;
}

void ScanSecondaryUpperCorners(void)
{
    //找左上，右上拐点
    for(int i = imgInfo.top + 2; i <= imgInfo.bottom - 5; i++)
    {

        /**************左上拐点**************/
        if(
             Left_Sideline[i] > 3 &&
             (
                  /*    *0
                      0100
                           */
                 (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
                  && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0000
                           */
                  (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                   && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && !Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      00010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0]&& Image_Use[i + 1][Left_Sideline[i] + 1] && !Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
//                   ||/*    *0
//                          01000
//                            */
// (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
// && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 3]
// && Image_Use[i + 1][Left_Sideline[i] - 2])

             )
             && white_width[i + 2] - white_width[i - 0] > 5
             && xielv_sideline(i, Left_Sideline[i], i + 1, Left_Sideline[i + 1], 'k') < 1
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Left_Sideline[i - 1] - Left_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Left_Sideline[i], i - 3, Left_Sideline[i - 3], 'k') - xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k')) > 0.5
//             && xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k') < 1
            //  && L_h_guai.flag == 0
            //  && i < L_l_guai.row
        )
        {
            L_h_guai.row = i + 1;
            L_h_guai.column = (uint8)Left_Sideline[i];
            L_h_guai.flag = 1;

//            if(R_h_guai.flag)
//                break;

        }

        /**************右上拐点**************/
        if(
             Right_Sideline[i] < LCDW_1 - 3 &&
             (
                   /*  0*
                       0010
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && !Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0100
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && !Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                      01000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 1] && !Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
//                    ||/*    0*
//                          00010
//                     */
// (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0]
// && Image_Use[i + 1][Right_Sideline[i] + 1] && !Image_Use[i + 1][Right_Sideline[i] + 3]
// && Image_Use[i + 1][Right_Sideline[i] + 2])

             )
             && white_width[i + 2] - white_width[i - 0] > 5
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Right_Sideline[i - 1] - Right_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
             && xielv_sideline(i, Right_Sideline[i], i + 1, Right_Sideline[i + 1], 'k') > -1
            //  && R_h_guai.flag == 0
            //  && i < R_l_guai.row
           )
        {
            R_h_guai.row = i + 1;
            R_h_guai.column = (uint8)Right_Sideline[i];
            R_h_guai.flag = 1;

//            if(L_h_guai.flag)
//                break;
        }

    }
}

void ScanSecondaryLowerCorners(void)
{
        //找左下，右下拐点
        for(int i = imgInfo.bottom - 3; i >= imgInfo.top + 3 && Flag.Huandao_L != 3 && Flag.Huandao_R != 3; i--)
        {
            if(L_h_guai.flag == 0)
                L_h_guai.row = 1;
            if(R_h_guai.flag == 0)
                R_h_guai.row = 1;
            /**************左下拐点**************/
            if(
                 Left_Sideline[i] > 3 &&
                 (
                      /*  0000
                            *0
                                */
                     (Image_Use[i][Left_Sideline[i] +0] && Image_Use[i - 1][Left_Sideline[i] + 0]
                      && Image_Use[i - 1][Left_Sideline[i] - 1] && Image_Use[i - 1][Left_Sideline[i] - 2]  && Image_Use[i - 1][Left_Sideline[i] -3])
                     ||
                     /*  0100
                           *0
                               */
                     (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] - 1]
                      && !Image_Use[i - 1][Left_Sideline[i] - 2] && Image_Use[i - 1][Left_Sideline[i] - 3])
                      ||
                      /*  0010
                            *0
                                */
                      (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i - 1][Left_Sideline[i] + 0] && !Image_Use[i - 1][Left_Sideline[i] - 1]
                       && Image_Use[i - 1][Left_Sideline[i] - 2] && Image_Use[i - 1][Left_Sideline[i] - 3])
                 )
                 /*上面三行的白行都比这一行多*/
                 && white_width[i - 2] - white_width[i + 1] > 10
                 && white_width[i - 2] > white_width[i] && white_width[i - 3] > white_width[i] && white_width[i - 4] > white_width[i]
                 && i > L_h_guai.row
                 && L_l_guai.flag == 0
            )
            {
                L_l_guai.row = i - 1;
                L_l_guai.column = (uint8)Left_Sideline[i];
                L_l_guai.flag = 1;

//                if(R_l_guai.flag)
//                    break;

            }

            /**************右下拐点**************/
            if(
                 Right_Sideline[i] < LCDW_1 - 3 &&
                 (
                         /*
                              0000
                              0*
                          */
                         (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0]
                         && Image_Use[i - 1][Right_Sideline[i] + 1] && Image_Use[i - 1][Right_Sideline[i] + 2] && Image_Use[i - 1][Right_Sideline[i] + 3])
                         ||
                         /*
                              0010
                              0*
                          */
                         (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] + 1]
                          && !Image_Use[i - 1][Right_Sideline[i] + 2] && Image_Use[i - 1][Right_Sideline[i] + 3])
                          ||
                          /*
                               0001
                               0*
                           */
                          (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] - 0] && Image_Use[i - 1][Right_Sideline[i] + 1]
                           && Image_Use[i - 1][Right_Sideline[i] + 2] && !Image_Use[i - 1][Right_Sideline[i] + 3])
                 )
                 /*上面三行的白行都比这一行多*/
                 && white_width[i - 2] > white_width[i] && white_width[i - 3] > white_width[i]
                 && white_width[i - 2] - white_width[i + 1] > 10
//                 && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
//                 && xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') > 1
                 && i > R_h_guai.row
                 && R_l_guai.flag == 0
            )
            {
                R_l_guai.row = i - 1;
                R_l_guai.column = (uint8)Right_Sideline[i];
                R_l_guai.flag = 1;

//                if(L_l_guai.flag)
//                    break;
            }
        }
}

void ApplySecondaryRoundaboutOverrides(void)
{
int l_guai_max=0,r_guai_max=93;//,l_guai_line,r_guai_line
if(Flag.Huandao_R == 4)
{
    for(int i = imgInfo.top + 2; i <54; i++)
    {
            if(Left_Sideline[i]> l_guai_max)//
            {
                l_guai_max = Left_Sideline[i];
                L_l_guai.row = i ;
            }
    }

                L_l_guai.column = (uint8)Left_Sideline[L_l_guai.row]+3;
                L_l_guai.flag = 1;
}
if(Flag.Huandao_L == 4)
{
    for(int i = imgInfo.top + 2; i <54; i++)
    {
            if(Right_Sideline[i]< r_guai_max)//
            {
                r_guai_max = Right_Sideline[i];
                R_l_guai.row = i ;
            }
    }

                R_l_guai.column = (uint8)Right_Sideline[R_l_guai.row]-3;
                R_l_guai.flag = 1;
}
}

}  // namespace

void Find_Guaidian1(void)
{
    ResetSecondaryCorners();
    ScanSecondaryUpperCorners();
    ScanSecondaryLowerCorners();
    ApplySecondaryRoundaboutOverrides();
}

struct Guaidian  L_h_guai1,  R_h_guai1;  //拐点信息结构体

void Find_l_h_Guaidian(void)
{
    L_h_guai1.flag = 0;
    R_h_guai1.flag = 0;
            L_h_guai1.row = 0;
            // L_h_guai1.column =0;
    //找左上，右上拐点
    for(int i = imgInfo.top + 2; i <= imgInfo.bottom - 5; i++)
    {

        /**************左上拐点**************/
        if(
             Left_Sideline[i] > 3 &&
             (
                  /*    *0
                      0100
                           */
                 (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0]
                  && Image_Use[i + 1][Left_Sideline[i] - 1] && !Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0000
                           */
                  (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                   && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      0010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] + 0] && !Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
                  ||
                  /*    *0
                      00010
                           */
                   (Image_Use[i][Left_Sideline[i] + 0]&& Image_Use[i + 1][Left_Sideline[i] + 1] && !Image_Use[i + 1][Left_Sideline[i] + 0] && Image_Use[i + 1][Left_Sideline[i] - 1]
                    && Image_Use[i + 1][Left_Sideline[i] - 2] && Image_Use[i + 1][Left_Sideline[i] - 3])
             )
             && white_width[i + 2] - white_width[i - 0] > 9
             && xielv_sideline(i, Left_Sideline[i], i + 1, Left_Sideline[i + 1], 'k') < 1
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Left_Sideline[i - 1] - Left_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Left_Sideline[i], i - 3, Left_Sideline[i - 3], 'k') - xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k')) > 0.5
//             && xielv_sideline(i, Left_Sideline[i], i + 2, Left_Sideline[i + 2], 'k') < 1
             && L_h_guai1.flag == 0
             && i < L_l_guai.row
        )
        {
            L_h_guai1.row = i + 1;
            L_h_guai1.column = (uint8)Left_Sideline[i];
            L_h_guai1.flag = 1;


        }


    }
}


void Find_r_h_Guaidian(void)
{
    R_h_guai1.flag = 0;
    L_h_guai1.flag = 0;
            R_h_guai1.row = 0;
            // R_h_guai1.column =0;
    //找左上，右上拐点
    for(int i = imgInfo.top + 2; i <= imgInfo.bottom - 5; i++)
    {
        /**************右上拐点**************/
        if(
             Right_Sideline[i] < LCDW_1 - 3 &&
             (
                   /*  0*
                       0010
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && !Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                       0100
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 0] && !Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
                   ||
                   /*  0*
                      01000
                            */
                  (Image_Use[i][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] - 1] && !Image_Use[i + 1][Right_Sideline[i] - 0] && Image_Use[i + 1][Right_Sideline[i] + 1]
                   && Image_Use[i + 1][Right_Sideline[i] + 2] && Image_Use[i + 1][Right_Sideline[i] + 3])
             )
             && white_width[i + 2] - white_width[i - 0] > 9
             && white_width[i + 3] > white_width[i] && white_width[i + 4] > white_width[i]
             //&& abs(Right_Sideline[i - 1] - Right_Sideline[i + 1]) > 8
//             && abs(xielv_sideline(i, Right_Sideline[i], i - 3, Right_Sideline[i - 3], 'k') - xielv_sideline(i, Right_Sideline[i], i + 2, Right_Sideline[i + 2], 'k')) > 0.5
             && xielv_sideline(i, Right_Sideline[i], i + 1, Right_Sideline[i + 1], 'k') > -1
             && R_h_guai1.flag == 0
             && i < R_l_guai.row
           )
        {
            R_h_guai1.row = i + 1;
            R_h_guai1.column = (uint8)Right_Sideline[i];
            R_h_guai1.flag = 1;

//            if(L_h_guai.flag)
//                break;
        }


    }
}
