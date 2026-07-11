#include "presentation/internal/presentation_stages.hpp"

void key_scan(void)
{
    presentation_scan_input();
}

void oled_show(void)
{
    presentation_render_oled_pages();
}
