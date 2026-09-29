#ifndef CCI_INVERTER_H
#define CCI_INVERTER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    CCI_INV_OK = 0,
    CCI_INV_ERR_ARGUMENT = -1,
    CCI_INV_ERR_CONNECT = -2,
    CCI_INV_ERR_IO = -3,
    CCI_INV_ERR_PROTOCOL = -4
} cci_inv_result_t;

typedef struct {
    char model[31];
    uint16_t model_id;
    uint16_t number_of_strings;
    uint16_t number_of_mppts;

    double rated_power_kw;
    double max_active_power_kw;
    double max_apparent_power_kva;
    double qmax_feed_to_grid_kvar;
    double qmax_absorb_from_grid_kvar;
} cci_inverter_info_t;

typedef struct {
    double dc_power_kw;

    double vab_v;
    double vbc_v;
    double vca_v;

    double ia_a;
    double ib_a;
    double ic_a;

    double active_power_kw;
    double reactive_power_kvar;
    double power_factor;
    double grid_frequency_hz;

    double efficiency_percent;
    double internal_temperature_c;

    uint16_t device_status_raw;
    uint16_t fault_code;

    uint64_t timestamp_ms;
    int valid;
} cci_inverter_measurements_t;

#ifdef __cplusplus
}
#endif

#endif
