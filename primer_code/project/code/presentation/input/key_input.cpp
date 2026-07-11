#include "presentation/internal/presentation_state.hpp"
#include "port/presentation_ports.hpp"
#include "zf_driver_delay.hpp"

void presentation_scan_input(void)
{
    key1_state=primer::port::presentation::DigitalKeyAt(0).get_level();//下
    key2_state=primer::port::presentation::DigitalKeyAt(1).get_level();//右
    key3_state=primer::port::presentation::DigitalKeyAt(2).get_level();//上
    key4_state=primer::port::presentation::DigitalKeyAt(3).get_level();//左
    key5_state =primer::port::presentation::DigitalKeyAt(4).get_level();//右按键中键
    key6_state =primer::port::presentation::DigitalKeyAt(5).get_level();//左按键中键
    key7_state = primer::port::presentation::AnalogKeyAt(0).convert();//右按键 左
    key8_state = primer::port::presentation::AnalogKeyAt(1).convert();//右按键 右

    if(key1_state==0&&key1_last_state==1)//下
    { Parameter_flag++; }
    if(key2_state==0&&key2_last_state==1)//右
    {
        primer::port::presentation::OperatorDisplay().clear(); oled_flag ++;
        if(oled_flag>4) oled_flag = 0;
        Parameter_flag=1;
    }
    if(key3_state==0&&key3_last_state==1)//上
    {
        Parameter_flag--;
        if(Parameter_flag < 1){ Parameter_flag = 1; }
    }
    if(key4_state==0&&key4_last_state==1) // 左
    {
        primer::port::presentation::OperatorDisplay().clear(); Parameter_flag = 0; oled_flag--;
        if(oled_flag<0) oled_flag = 0;
    }
    if(key5_state==0&&key5_last_state==1) // //右按键中键
    { primer::port::presentation::SetRunFlag(2); }//停车标志位
    if(key6_state==0&&key6_last_state==1) // 左按键中键
    {
        primer::port::presentation::OperatorDisplay().clear(); system_delay_ms(1000);
        for(int i=550;i<=1000;i++)
        {
            primer::port::presentation::SetEscDuty(i);
            system_delay_ms(4);
        }
        primer::port::presentation::SetRunFlag(1);//发车标志位
    }
    if(key7_state<4000&&key7_last_state>=4000) // 右按键 左
    {
        if(Parameter_flag == 1&&oled_flag==4){primer::port::presentation::ObserveParameters().debug_rgb_r_min-=5;}
        if(Parameter_flag == 2&&oled_flag==4){primer::port::presentation::ObserveParameters().debug_rgb_rg_diff-=5;}
        if(Parameter_flag == 3&&oled_flag==4){primer::port::presentation::ObserveParameters().debug_rgb_rb_diff-=5;}
        primer::port::presentation::SaveParameters();
    }
    if(key8_state<4000&&key8_last_state>=4000) // 右按键 右
    {
        if(Parameter_flag == 1&&oled_flag==4){primer::port::presentation::ObserveParameters().debug_rgb_r_min+=5;}
        if(Parameter_flag == 2&&oled_flag==4){primer::port::presentation::ObserveParameters().debug_rgb_rg_diff+=5;}
        if(Parameter_flag == 3&&oled_flag==4){primer::port::presentation::ObserveParameters().debug_rgb_rb_diff+=5;}
        primer::port::presentation::SaveParameters();
    }
    key1_last_state=key1_state;
    key2_last_state=key2_state;
    key3_last_state=key3_state;
    key4_last_state=key4_state;
    key5_last_state=key5_state;
    key6_last_state=key6_state;
    key7_last_state=key7_state;
    key8_last_state=key8_state;
}
