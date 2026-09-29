#include "huawei_sun2000.h"

#include <stdint.h>
#include <string.h>
#include <time.h>

/* Huawei Solar Inverter Modbus Interface Definitions (V3.0). */
#define REG_MODEL                30000
#define REG_MODEL_ID             30070
#define REG_DC_POWER             32064

static uint32_t decode_u32(const uint16_t *r)
{
    return ((uint32_t)r[0] << 16) | (uint32_t)r[1];
}

static int32_t decode_i32(const uint16_t *r)
{
    return (int32_t)decode_u32(r);
}

static int16_t decode_i16(uint16_t r)
{
    return (int16_t)r;
}

static uint64_t timestamp_ms(void)
{
    return (uint64_t)time(NULL) * 1000ULL;
}

static void decode_ascii(
    const uint16_t *regs,
    size_t register_count,
    char *dest,
    size_t dest_size
)
{
    size_t i;
    size_t out = 0;

    if (!dest || dest_size == 0)
        return;

    for (i = 0; i < register_count && out + 1 < dest_size; ++i) {
        unsigned char hi = (unsigned char)((regs[i] >> 8) & 0xff);
        unsigned char lo = (unsigned char)(regs[i] & 0xff);

        if (hi && out + 1 < dest_size)
            dest[out++] = (char)hi;
        if (lo && out + 1 < dest_size)
            dest[out++] = (char)lo;
    }

    dest[out] = '\0';
}

int huawei_sun2000_init(
    huawei_sun2000_t *driver,
    modbus_transport_t *transport
)
{
    if (!driver || !transport)
        return CCI_INV_ERR_ARGUMENT;

    driver->transport = transport;
    return CCI_INV_OK;
}

int huawei_sun2000_read_info(
    huawei_sun2000_t *driver,
    cci_inverter_info_t *info
)
{
    uint16_t model[15];
    uint16_t r[13];

    if (!driver || !driver->transport || !info)
        return CCI_INV_ERR_ARGUMENT;

    memset(info, 0, sizeof(*info));

    if (modbus_transport_read_holding(
            driver->transport, REG_MODEL, 15, model) != 0)
        return CCI_INV_ERR_IO;

    if (modbus_transport_read_holding(
            driver->transport, REG_MODEL_ID, 13, r) != 0)
        return CCI_INV_ERR_IO;

    decode_ascii(model, 15, info->model, sizeof(info->model));

    info->model_id = r[0];
    info->number_of_strings = r[1];
    info->number_of_mppts = r[2];

    info->rated_power_kw =
        (double)decode_u32(&r[3]) / 1000.0;

    info->max_active_power_kw =
        (double)decode_u32(&r[5]) / 1000.0;

    info->max_apparent_power_kva =
        (double)decode_u32(&r[7]) / 1000.0;

    info->qmax_feed_to_grid_kvar =
        (double)decode_i32(&r[9]) / 1000.0;

    info->qmax_absorb_from_grid_kvar =
        (double)decode_i32(&r[11]) / 1000.0;

    return CCI_INV_OK;
}

int huawei_sun2000_read_measurements(
    huawei_sun2000_t *driver,
    cci_inverter_measurements_t *m
)
{
    /* 32064..32090, one FC03 request. */
    uint16_t r[27];

    if (!driver || !driver->transport || !m)
        return CCI_INV_ERR_ARGUMENT;

    memset(m, 0, sizeof(*m));

    if (modbus_transport_read_holding(
            driver->transport, REG_DC_POWER, 27, r) != 0) {
        m->valid = 0;
        return CCI_INV_ERR_IO;
    }

    m->dc_power_kw = (double)decode_i32(&r[0]) / 1000.0;

    m->vab_v = (double)r[2] / 10.0;
    m->vbc_v = (double)r[3] / 10.0;
    m->vca_v = (double)r[4] / 10.0;

    m->ia_a = (double)decode_i32(&r[8]) / 1000.0;
    m->ib_a = (double)decode_i32(&r[10]) / 1000.0;
    m->ic_a = (double)decode_i32(&r[12]) / 1000.0;

    m->active_power_kw = (double)decode_i32(&r[16]) / 1000.0;
    m->reactive_power_kvar = (double)decode_i32(&r[18]) / 1000.0;

    m->power_factor = (double)decode_i16(r[20]) / 1000.0;
    m->grid_frequency_hz = (double)r[21] / 100.0;

    m->efficiency_percent = (double)r[22] / 100.0;
    m->internal_temperature_c =
        (double)decode_i16(r[23]) / 10.0;

    m->device_status_raw = r[25];
    m->fault_code = r[26];

    m->timestamp_ms = timestamp_ms();
    m->valid = 1;

    return CCI_INV_OK;
}
