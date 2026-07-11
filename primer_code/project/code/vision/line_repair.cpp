#include "internal/dependencies/line_repair_dependencies.hpp"

#include <cstdlib>

namespace {

void RepairFourCorners()
{
    if(L_l_guai.flag && L_h_guai.flag && R_l_guai.flag && R_h_guai.flag && abs(R_h_guai.column - L_h_guai.column) > 5)
    {
        float kl, kr, bl, br;
        if(L_h_guai.row < L_l_guai.row)
        {
            kl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'k');
            bl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'b');
            for (int i = L_h_guai.row; i <= L_l_guai.row; i++)
            {
                Left_Sideline[i] = kl * i + bl;
                Left_Sideline_flag[i] = 1;
            }
            Flag.Buxian = 1;
        }
        if(R_h_guai.row < R_l_guai.row)
        {
            kr = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'k');
            br = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'b');
            for (int i = R_h_guai.row; i <= R_l_guai.row; i++)
            {
                Right_Sideline[i] = kr * i + br;
                Right_Sideline_flag[i] = 1;
            }
            Flag.Buxian = 1;
        }
    }
}

bool RepairLeftPairAndRightHigh()
{
    if(L_l_guai.flag && L_h_guai.flag && R_h_guai.flag && abs(R_h_guai.column - L_h_guai.column) > 5)
    {
    float kl, kr, bl, br;
    if(L_h_guai.row < L_l_guai.row)
    {
        kl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'k');
        bl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'b');
        for (int i = L_h_guai.row; i <= L_l_guai.row; i++)
        {
            Left_Sideline[i] = kl * i + bl;
            Left_Sideline_flag[i] = 1;
        }
        Flag.Buxian = 1;
    }
    if(R_h_guai.row < L_l_guai.row)
    {
        kr = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 5, Right_Sideline[imgInfo.bottom - 5], 'k');
        br = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 5, Right_Sideline[imgInfo.bottom - 5], 'b');
        for (int i = R_h_guai.row; i <= imgInfo.bottom - 5; i++)
        {
            Right_Sideline[i] = kr * i + br;
            Right_Sideline_flag[i] = 1;
            if ((Right_Sideline[i] == Right_Sideline[i + 1] && Right_Sideline[i] == Right_Sideline[i + 2])
                || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]) break;
        }
        Flag.Buxian = 1;
    }
        return true;
    }
    return false;
}

bool RepairRightPairAndLeftHigh()
{
    if(R_l_guai.flag && R_h_guai.flag && L_h_guai.flag && abs(R_h_guai.column - L_h_guai.column) > 5)
    {
    float kl, kr, bl, br;
    if(L_h_guai.row < R_l_guai.row)
    {
        kl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 5, Left_Sideline[imgInfo.bottom - 5], 'k');
        bl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 5, Left_Sideline[imgInfo.bottom - 5], 'b');
        for (int i = L_h_guai.row; i <= imgInfo.bottom - 5; i++)
        {
            Left_Sideline[i] = kl * i + bl;
            Left_Sideline_flag[i] = 1;
            if((Left_Sideline[i] == Left_Sideline[i + 1] && Left_Sideline[i] == Left_Sideline[i + 2])
                || !Image_Use[i + 1][(uint8)(kl * (i + 1) + bl)]) break;
        }
        Flag.Buxian = 1;
    }
    if(R_h_guai.row < R_l_guai.row)
    {
        kr = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'k');
        br = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'b');
        for (int i = R_h_guai.row; i <= R_l_guai.row; i++)
        {
            Right_Sideline[i] = kr * i + br;
            Right_Sideline_flag[i] = 1;
        }
        Flag.Buxian = 1;
    }
        return true;
    }
    return false;
}

bool RepairBothHighCorners()
{
    if(R_h_guai.flag && L_h_guai.flag && abs(R_h_guai.column - L_h_guai.column) > 5)
    {
    float kl, kr, bl, br;
    kl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 5, Left_Sideline[imgInfo.bottom - 5] - 3, 'k');
    bl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 5, Left_Sideline[imgInfo.bottom - 5] - 3, 'b');
    for (int i = L_h_guai.row; i <= imgInfo.bottom - 5; i++)
    {
        Left_Sideline[i] = kl * i + bl;
        Left_Sideline_flag[i] = 1;
        if((Left_Sideline[i] == Left_Sideline[i + 1] && Left_Sideline[i] == Left_Sideline[i + 2])
            || !Image_Use[i + 1][(uint8)(kl * (i + 1) + bl)]) break;
    }
    kr = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 5, Right_Sideline[imgInfo.bottom - 5] - 3, 'k');
    br = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 5, Right_Sideline[imgInfo.bottom - 5] - 3, 'b');
    for (int i = R_h_guai.row; i < imgInfo.bottom - 5; i++)
    {
        Right_Sideline[i] = kr * i + br;
        Right_Sideline_flag[i] = 1;
        if ((Right_Sideline[i] == Right_Sideline[i + 1] && Right_Sideline[i] ==  Right_Sideline[i + 2])
            || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]) break;
    }
    Flag.Buxian = 1;
        return true;
    }
    return false;
}

bool RepairLeftCornerPair()
{
    if(L_l_guai.flag && L_h_guai.flag)
    {
    float kl, bl;
    if(L_h_guai.row < L_l_guai.row)
    {
        kl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'k');
        bl = xielv_sideline(L_l_guai.row, L_l_guai.column, L_h_guai.row, L_h_guai.column, 'b');
        for (int i = L_h_guai.row; i <= L_l_guai.row; i++)
        {
            Left_Sideline[i] = kl * i + bl;
            Left_Sideline_flag[i] = 1;
        }
    }
    Flag.Buxian = 1;
        return true;
    }
    return false;
}

bool RepairRightCornerPair()
{
    if(R_l_guai.flag && R_h_guai.flag)
    {
    float kr, br;
    if(R_h_guai.row < R_l_guai.row)
    {
        kr = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'k');
        br = xielv_sideline(R_l_guai.row, R_l_guai.column, R_h_guai.row, R_h_guai.column, 'b');
        for (int i = R_h_guai.row; i <= R_l_guai.row; i++)
        {
            Right_Sideline[i] = kr * i + br;
            Right_Sideline_flag[i] = 1;
        }
    }
    Flag.Buxian = 1;
        return true;
    }
    return false;
}

bool RepairLeftHighCorner()
{
    if(L_h_guai.flag&&imgInfo.Both_lose>0)
    {
    float kl, bl;
    kl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 3, Left_Sideline[imgInfo.bottom - 3], 'k');
    bl = xielv_sideline(L_h_guai.row, L_h_guai.column, imgInfo.bottom - 3, Left_Sideline[imgInfo.bottom - 3], 'b');
    for (int i = L_h_guai.row; i <= imgInfo.bottom - 5; i++)
    {
        Left_Sideline[i] = kl * i + bl;
        Left_Sideline_flag[i] = 1;
        if((Left_Sideline[i] == Left_Sideline[i + 1] && Left_Sideline[i] == Left_Sideline[i + 2])
            || !Image_Use[i + 1][(uint8)(kl * (i + 1) + bl)]) break;
    }
    Flag.Buxian = 1;
        return true;
    }
    return false;
}

bool RepairRightHighCorner()
{
    if(R_h_guai.flag&&imgInfo.Both_lose>0)
    {
    float kr, br;
    kr = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 3, Right_Sideline[imgInfo.bottom - 3], 'k');
    br = xielv_sideline(R_h_guai.row, R_h_guai.column, imgInfo.bottom - 3, Right_Sideline[imgInfo.bottom - 3], 'b');
    for (int i = R_h_guai.row; i < imgInfo.bottom - 5; i++)
    {
        Right_Sideline[i] = kr * i + br;
        Right_Sideline_flag[i] = 1;
        if ((Right_Sideline[i] == Right_Sideline[i + 1] && Right_Sideline[i] ==  Right_Sideline[i + 2])
            || !Image_Use[i + 1][(uint8)(kr * (i + 1) + br)]) break;
    }
    Flag.Buxian = 1;
        return true;
    }
    return false;
}

}  // namespace

void Buxian(void)
{
    Flag.Buxian = 0;
    RepairFourCorners();
    if (!RepairLeftPairAndRightHigh()
        && !RepairRightPairAndLeftHigh()
        && !RepairBothHighCorners()
        && !RepairLeftCornerPair()
        && !RepairRightCornerPair()
        && !RepairLeftHighCorner()
        && !RepairRightHighCorner())
    {
        Flag.Buxian = 0;
    }
}
