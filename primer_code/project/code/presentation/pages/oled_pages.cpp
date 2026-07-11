#include "presentation/internal/presentation_pages.hpp"

void presentation_render_oled_pages(void)
{
    primer::presentation::RenderRedDebugPage();
    primer::presentation::RenderPage3();
    primer::presentation::RenderPage2();
    primer::presentation::RenderPage1();
    primer::presentation::RenderPage0();
}
