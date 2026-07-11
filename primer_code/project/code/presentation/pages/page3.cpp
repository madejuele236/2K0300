#include "presentation/internal/presentation_pages.hpp"
#include "presentation/internal/presentation_state.hpp"
#include "presentation/internal/page_common.hpp"
#include "port/presentation_ports.hpp"

#include <cstdio>

namespace primer::presentation {

void RenderPage3(void)
{
if(oled_flag == 3)
{


    internal::RenderPageFooter();
}
}

}  // namespace primer::presentation
