#include "presentation/internal/presentation_pages.hpp"
#include "presentation/internal/presentation_state.hpp"
#include "port/presentation_ports.hpp"

#include <cstdio>

namespace primer::presentation {

void RenderPage3(void)
{
if(oled_flag == 3)
{


    sprintf(txt,"Parameter:%d",Parameter_flag);
    primer::port::presentation::OperatorDisplay().show_string(0,300,txt);

    sprintf(txt,"oled_page:%d",oled_flag);
    primer::port::presentation::OperatorDisplay().show_string(130,300,txt);
}
}

}  // namespace primer::presentation
