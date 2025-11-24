#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <time.h>

#include "clock.h"
#include "cts.h"

static time_t boot_time_epoch = 0;  // Boot reference time (Unix time)

// Function to print the current time
void print_clock()
{
    struct tm datetime;
    clock_get_datetime(&datetime);
    printk("Current time: %02d:%02d:%02d, %04d-%02d-%02d\n",
        datetime.tm_hour, datetime.tm_min, datetime.tm_sec,
        datetime.tm_year + 1900, datetime.tm_mon + 1, datetime.tm_mday);
}

// Retrieves the current date and time based on system uptime
void clock_get_datetime(struct tm *datetime)
{
    if (datetime == NULL) {
        printk("Error: clock_get_datetime received NULL pointer\n");
        return;
    }

    time_t current_time = boot_time_epoch + (k_uptime_get() / 1000);

    struct tm temp_tm;
    struct tm *local_tm = gmtime_r(&current_time, &temp_tm);  // Use thread-safe gmtime_r()
    if (local_tm == NULL) {
        printk("Error: gmtime_r() returned NULL\n");
        return;
    }

    *datetime = *local_tm;  // Copy converted time
}

// Initializes the system clock
void clock_init()
{
    struct tm cts_time;
    cts_init();  // Perform CTS synchronization

    // Set initial time from CTS (default: January 1, 2024)
    cts_time.tm_year = 2024 - 1900;
    cts_time.tm_mon = 0;  // January
    cts_time.tm_mday = 1;
    cts_time.tm_hour = 0;
    cts_time.tm_min = 0;
    cts_time.tm_sec = 0;

    sync_cts_to_time(&cts_time);  // Synchronize time
}

// Updates the clock with a new date and time
void clock_set_datetime(struct tm* datetime)
{
    time_t set_time = mktime(datetime);
    time_t current_uptime = k_uptime_get() / 1000;

    // Adjust boot_time_epoch so that current_time = set_time
    boot_time_epoch = set_time - current_uptime;
    
    printk("Local clock set to: %02d:%02d:%02d, %04d-%02d-%02d\n",
        datetime->tm_hour, datetime->tm_min, datetime->tm_sec,
        datetime->tm_year + 1900, datetime->tm_mon + 1, datetime->tm_mday);
}

// Returns the elapsed seconds since boot
time_t clock_get_ticks()
{
    return k_uptime_get() / 1000;
}
