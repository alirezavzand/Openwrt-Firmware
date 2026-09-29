#ifndef _WIN32
#define _DEFAULT_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include "cci_inverter.h"
#include "huawei_sun2000.h"
#include "modbus_transport.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <windows.h>
static void sleep_ms(unsigned int ms) { Sleep(ms); }
#define DEFAULT_PORT "COM2"
#else
#include <time.h>
static void sleep_ms(unsigned int ms)
{
    struct timespec ts;
    ts.tv_sec = ms / 1000U;
    ts.tv_nsec = (long)(ms % 1000U) * 1000000L;
    nanosleep(&ts, NULL);
}
#define DEFAULT_PORT "/dev/ttyUSB0"
#endif

static void print_info(const cci_inverter_info_t *i)
{
    printf("\n--- Inverter info ---\n");
    printf("Model     : %s\n", i->model);
    printf("Model ID  : %u\n", (unsigned)i->model_id);
    printf("Strings   : %u\n", (unsigned)i->number_of_strings);
    printf("MPPTs     : %u\n", (unsigned)i->number_of_mppts);
    printf("Pn        : %.3f kW\n", i->rated_power_kw);
    printf("Pmax      : %.3f kW\n", i->max_active_power_kw);
    printf("Smax(inv) : %.3f kVA\n", i->max_apparent_power_kva);
}

static void print_measurements(const cci_inverter_measurements_t *m)
{
    printf(
        "P=%7.2f kW | Q=%7.2f kvar | "
        "V=%6.1f/%6.1f/%6.1f V | "
        "I=%6.1f/%6.1f/%6.1f A | "
        "f=%5.2f Hz | PF=%6.3f | "
        "status=0x%04X | fault=%u\n",
        m->active_power_kw,
        m->reactive_power_kvar,
        m->vab_v, m->vbc_v, m->vca_v,
        m->ia_a, m->ib_a, m->ic_a,
        m->grid_frequency_hz,
        m->power_factor,
        (unsigned)m->device_status_raw,
        (unsigned)m->fault_code
    );
}

int main(int argc, char **argv)
{
    const char *port = DEFAULT_PORT;
    int baud = 9600;
    int slave = 1;

    modbus_rtu_config_t cfg;
    modbus_transport_t *transport;
    huawei_sun2000_t inverter;
    cci_inverter_info_t info;
    cci_inverter_measurements_t m;

    if (argc >= 2)
        port = argv[1];
    if (argc >= 3)
        baud = atoi(argv[2]);
    if (argc >= 4)
        slave = atoi(argv[3]);

    cfg.serial_port = port;
    cfg.baud_rate = baud;
    cfg.parity = 'N';
    cfg.data_bits = 8;
    cfg.stop_bits = 1;
    cfg.slave_id = slave;
    cfg.response_timeout_ms = 1500;

    printf("CCI Huawei SUN2000 Modbus RTU reader\n");
    printf("Port     : %s\n", cfg.serial_port);
    printf("Serial   : %d 8%c%d\n",
           cfg.baud_rate, cfg.parity, cfg.stop_bits);
    printf("Slave ID : %d\n", cfg.slave_id);

    transport = modbus_transport_open_rtu(&cfg);

    if (!transport) {
        fprintf(stderr, "Failed to open serial transport.\n");
        return 1;
    }

    if (huawei_sun2000_init(&inverter, transport) != CCI_INV_OK) {
        fprintf(stderr, "Huawei driver init failed.\n");
        modbus_transport_close(transport);
        return 1;
    }

    if (huawei_sun2000_read_info(&inverter, &info) != CCI_INV_OK) {
        fprintf(
            stderr,
            "Failed to read inverter info: %s\n",
            modbus_transport_last_error(transport)
        );
        modbus_transport_close(transport);
        return 1;
    }

    print_info(&info);

    if (info.model_id != 150) {
        fprintf(
            stderr,
            "WARNING: expected SUN2000-100KTL-M2 model ID 150, got %u\n",
            (unsigned)info.model_id
        );
    }

    printf("\n--- Live measurements ---\n");

    for (;;) {
        if (huawei_sun2000_read_measurements(
                &inverter, &m) == CCI_INV_OK) {
            print_measurements(&m);
        } else {
            fprintf(
                stderr,
                "Read failed: %s\n",
                modbus_transport_last_error(transport)
            );
        }

        sleep_ms(1000);
    }

    return 0;
}
