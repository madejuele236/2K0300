#include "../internal/dependencies/midline_dependencies.hpp"

int Mid_Line[LCDH_0] = {LCDW_0 / 2};     // 中线数组，初始化所有元素为屏幕宽度的一半

/***************************************************找中线**********************************************************/
void Find_Midline(){
//   if(imgInfo.L_loselineSum==0&&imgInfo.R_loselineSum==0){

    for(uint16 i = imgInfo.bottom-1;i>imgInfo.top;i--){
      Mid_Line[i] = (Left_Sideline[i] + Right_Sideline[i])/2;
    }
//   }
        for(uint16 i = imgInfo.bottom - 1; i > imgInfo.top; i--)
        {
            Last_Mid_Line[i] = Mid_Line[i];
        }
}
