#include "app.h"
#include "hal_stm32f405.h"

int main(void)
{
    hal_init(); /* clocks, pins, peripherals, interrupts */
    app_init(); /* sensors, log, GPS configuration, watchdog */
    for (;;)
        app_step();
}
