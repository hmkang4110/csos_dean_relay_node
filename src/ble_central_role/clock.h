#include <zephyr/kernel.h>
#include <time.h>

void clock_init();

time_t clock_get_ticks();

void clock_get_datetime(struct tm *datetime);
void clock_set_datetime(struct tm *datetime);