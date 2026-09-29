#include "modbus_transport.h"

#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct modbus_transport {
    HANDLE handle;
    int slave_id;
    int timeout_ms;
    char last_error[256];
};

static uint16_t crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xffff;
    size_t pos;
    int i;

    for (pos = 0; pos < length; ++pos) {
        crc ^= data[pos];

        for (i = 0; i < 8; ++i) {
            if (crc & 1)
                crc = (uint16_t)((crc >> 1) ^ 0xa001);
            else
                crc >>= 1;
        }
    }

    return crc;
}

static int read_exact(
    modbus_transport_t *t,
    uint8_t *buf,
    size_t wanted
)
{
    size_t got = 0;
    DWORD started = GetTickCount();

    while (got < wanted) {
        DWORD nread = 0;

        if ((int)(GetTickCount() - started) >= t->timeout_ms) {
            snprintf(
                t->last_error,
                sizeof(t->last_error),
                "Modbus RTU timeout (%u/%u bytes)",
                (unsigned)got,
                (unsigned)wanted
            );
            return -1;
        }

        if (!ReadFile(
                t->handle,
                buf + got,
                (DWORD)(wanted - got),
                &nread,
                NULL
            )) {
            snprintf(
                t->last_error,
                sizeof(t->last_error),
                "ReadFile failed: Win32 error %lu",
                (unsigned long)GetLastError()
            );
            return -1;
        }

        if (nread == 0) {
            Sleep(1);
            continue;
        }

        got += (size_t)nread;
    }

    return 0;
}

modbus_transport_t *modbus_transport_open_rtu(
    const modbus_rtu_config_t *cfg
)
{
    modbus_transport_t *t;
    DCB dcb;
    COMMTIMEOUTS timeouts;
    char port_name[64];

    if (!cfg || !cfg->serial_port ||
        cfg->slave_id < 1 || cfg->slave_id > 247)
        return NULL;

    t = (modbus_transport_t *)calloc(1, sizeof(*t));
    if (!t)
        return NULL;

    snprintf(
        port_name,
        sizeof(port_name),
        "\\\\.\\%s",
        cfg->serial_port
    );

    t->handle = CreateFileA(
        port_name,
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        0,
        NULL
    );

    if (t->handle == INVALID_HANDLE_VALUE) {
        free(t);
        return NULL;
    }

    memset(&dcb, 0, sizeof(dcb));
    dcb.DCBlength = sizeof(dcb);

    if (!GetCommState(t->handle, &dcb)) {
        CloseHandle(t->handle);
        free(t);
        return NULL;
    }

    dcb.BaudRate = (DWORD)cfg->baud_rate;
    dcb.ByteSize = (BYTE)cfg->data_bits;
    dcb.StopBits =
        cfg->stop_bits == 2 ? TWOSTOPBITS : ONESTOPBIT;

    if (cfg->parity == 'E') {
        dcb.Parity = EVENPARITY;
        dcb.fParity = TRUE;
    } else if (cfg->parity == 'O') {
        dcb.Parity = ODDPARITY;
        dcb.fParity = TRUE;
    } else {
        dcb.Parity = NOPARITY;
        dcb.fParity = FALSE;
    }

    dcb.fBinary = TRUE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;
    dcb.fOutX = FALSE;
    dcb.fInX = FALSE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;

    if (!SetCommState(t->handle, &dcb)) {
        CloseHandle(t->handle);
        free(t);
        return NULL;
    }

    memset(&timeouts, 0, sizeof(timeouts));
    timeouts.ReadIntervalTimeout = 20;
    timeouts.ReadTotalTimeoutConstant = 20;
    timeouts.WriteTotalTimeoutConstant = 1000;

    if (!SetCommTimeouts(t->handle, &timeouts)) {
        CloseHandle(t->handle);
        free(t);
        return NULL;
    }

    PurgeComm(t->handle, PURGE_RXCLEAR | PURGE_TXCLEAR);

    t->slave_id = cfg->slave_id;
    t->timeout_ms =
        cfg->response_timeout_ms > 0
        ? cfg->response_timeout_ms
        : 1000;

    snprintf(t->last_error, sizeof(t->last_error), "OK");
    return t;
}

void modbus_transport_close(modbus_transport_t *t)
{
    if (!t)
        return;

    if (t->handle != INVALID_HANDLE_VALUE)
        CloseHandle(t->handle);

    free(t);
}

int modbus_transport_read_holding(
    modbus_transport_t *t,
    int address,
    int count,
    uint16_t *dest
)
{
    uint8_t request[8];
    uint8_t header[3];
    uint8_t response[260];
    uint16_t request_crc;
    uint16_t calculated_crc;
    uint16_t received_crc;
    size_t total;
    DWORD written = 0;
    int i;

    if (!t || !dest ||
        address < 0 || address > 65535 ||
        count < 1 || count > 125) {
        return -1;
    }

    request[0] = (uint8_t)t->slave_id;
    request[1] = 0x03;
    request[2] = (uint8_t)((address >> 8) & 0xff);
    request[3] = (uint8_t)(address & 0xff);
    request[4] = (uint8_t)((count >> 8) & 0xff);
    request[5] = (uint8_t)(count & 0xff);

    request_crc = crc16(request, 6);
    request[6] = (uint8_t)(request_crc & 0xff);
    request[7] = (uint8_t)((request_crc >> 8) & 0xff);

    PurgeComm(t->handle, PURGE_RXCLEAR);

    if (!WriteFile(
            t->handle,
            request,
            (DWORD)sizeof(request),
            &written,
            NULL
        )) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "WriteFile failed: Win32 error %lu",
            (unsigned long)GetLastError()
        );
        return -1;
    }

    FlushFileBuffers(t->handle);

    if (read_exact(t, header, 3) != 0)
        return -1;

    if (header[0] != (uint8_t)t->slave_id) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "Unexpected slave ID %u",
            (unsigned)header[0]
        );
        return -1;
    }

    if (header[1] & 0x80) {
        uint8_t tail[2];

        if (read_exact(t, tail, 2) != 0)
            return -1;

        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "Modbus exception code %u",
            (unsigned)header[2]
        );
        return -1;
    }

    if (header[1] != 0x03 ||
        header[2] != (uint8_t)(count * 2)) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "Unexpected Modbus response"
        );
        return -1;
    }

    response[0] = header[0];
    response[1] = header[1];
    response[2] = header[2];

    total = 3U + (size_t)header[2] + 2U;

    if (read_exact(t, &response[3], total - 3U) != 0)
        return -1;

    calculated_crc = crc16(response, total - 2U);

    received_crc =
        (uint16_t)response[total - 2U] |
        ((uint16_t)response[total - 1U] << 8);

    if (calculated_crc != received_crc) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "CRC mismatch"
        );
        return -1;
    }

    for (i = 0; i < count; ++i) {
        dest[i] =
            ((uint16_t)response[3 + 2 * i] << 8) |
            (uint16_t)response[4 + 2 * i];
    }

    snprintf(t->last_error, sizeof(t->last_error), "OK");
    return 0;
}

const char *modbus_transport_last_error(modbus_transport_t *t)
{
    return t ? t->last_error : "transport is NULL";
}

#endif
