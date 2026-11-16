/*
 * health.h - deciding when something is broken, and when to try again.
 */
#ifndef VAYU_HEALTH_H
#define VAYU_HEALTH_H

#include <stdbool.h>
#include <stdint.h>

/* Per-sensor bookkeeping. One bad transaction is noise; SENSOR_ERR_LIMIT in a
 * row means the sensor is gone. A failed sensor is re-initialised every
 * SENSOR_RETRY_MS, so a sensor that comes back is used again. */
typedef struct {
    uint8_t  errors;
    bool     failed;
    uint32_t next_retry_ms;
} dev_health_t;

void dev_health_reset(dev_health_t *h);
void dev_health_fail(dev_health_t *h, uint32_t now);
void dev_health_report(dev_health_t *h, bool ok, uint32_t now);
bool dev_health_retry_due(dev_health_t *h, uint32_t now); /* also books the next retry */

void health_init(void);
void health_task(uint32_t now); /* builds V.faults and reports every change */

#endif
