#ifndef HUAWEI_SUN2000_H
#define HUAWEI_SUN2000_H

#include "cci_inverter.h"
#include "modbus_transport.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    modbus_transport_t *transport;
} huawei_sun2000_t;

int huawei_sun2000_init(
    huawei_sun2000_t *driver,
    modbus_transport_t *transport
);

int huawei_sun2000_read_info(
    huawei_sun2000_t *driver,
    cci_inverter_info_t *info
);

int huawei_sun2000_read_measurements(
    huawei_sun2000_t *driver,
    cci_inverter_measurements_t *m
);

#ifdef __cplusplus
}
#endif

#endif
