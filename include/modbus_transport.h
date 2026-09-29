#ifndef MODBUS_TRANSPORT_H
#define MODBUS_TRANSPORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct modbus_transport modbus_transport_t;

typedef struct {
    const char *serial_port;   /* Windows: COM2, OpenWrt: /dev/ttyS1 or /dev/ttyUSB0 */
    int baud_rate;
    char parity;               /* N, E, O */
    int data_bits;             /* normally 8 */
    int stop_bits;             /* 1 or 2 */
    int slave_id;              /* 1..247 */
    int response_timeout_ms;
} modbus_rtu_config_t;

modbus_transport_t *modbus_transport_open_rtu(const modbus_rtu_config_t *cfg);
void modbus_transport_close(modbus_transport_t *transport);

int modbus_transport_read_holding(
    modbus_transport_t *transport,
    int address,
    int count,
    uint16_t *dest
);

const char *modbus_transport_last_error(modbus_transport_t *transport);

#ifdef __cplusplus
}
#endif

#endif
