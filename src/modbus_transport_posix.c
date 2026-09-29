#include "modbus_transport.h"

#ifndef _WIN32

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

struct modbus_transport {
    int fd;
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

static speed_t baud_to_termios(int baud)
{
    switch (baud) {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
#ifdef B57600
    case 57600: return B57600;
#endif
#ifdef B115200
    case 115200: return B115200;
#endif
    default: return (speed_t)0;
    }
}

static int read_exact(
    modbus_transport_t *t,
    uint8_t *buf,
    size_t wanted
)
{
    size_t got = 0;

    while (got < wanted) {
        struct pollfd pfd;
        ssize_t n;
        int rc;

        pfd.fd = t->fd;
        pfd.events = POLLIN;
        pfd.revents = 0;

        rc = poll(&pfd, 1, t->timeout_ms);

        if (rc == 0) {
            snprintf(
                t->last_error,
                sizeof(t->last_error),
                "Modbus RTU timeout (%u/%u bytes received)",
                (unsigned)got,
                (unsigned)wanted
            );
            return -1;
        }

        if (rc < 0) {
            snprintf(
                t->last_error,
                sizeof(t->last_error),
                "poll failed: %s",
                strerror(errno)
            );
            return -1;
        }

        n = read(t->fd, buf + got, wanted - got);

        if (n < 0) {
            if (errno == EINTR)
                continue;

            snprintf(
                t->last_error,
                sizeof(t->last_error),
                "read failed: %s",
                strerror(errno)
            );
            return -1;
        }

        if (n == 0)
            continue;

        got += (size_t)n;
    }

    return 0;
}

static int write_all(
    modbus_transport_t *t,
    const uint8_t *buf,
    size_t length
)
{
    size_t sent = 0;

    while (sent < length) {
        ssize_t n = write(t->fd, buf + sent, length - sent);

        if (n < 0) {
            if (errno == EINTR)
                continue;

            snprintf(
                t->last_error,
                sizeof(t->last_error),
                "write failed: %s",
                strerror(errno)
            );
            return -1;
        }

        sent += (size_t)n;
    }

    if (tcdrain(t->fd) != 0) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "tcdrain failed: %s",
            strerror(errno)
        );
        return -1;
    }

    return 0;
}

modbus_transport_t *modbus_transport_open_rtu(
    const modbus_rtu_config_t *cfg
)
{
    modbus_transport_t *t;
    struct termios tio;
    speed_t speed;

    if (!cfg || !cfg->serial_port)
        return NULL;

    if (cfg->slave_id < 1 || cfg->slave_id > 247)
        return NULL;

    speed = baud_to_termios(cfg->baud_rate);
    if (speed == 0)
        return NULL;

    t = (modbus_transport_t *)calloc(1, sizeof(*t));
    if (!t)
        return NULL;

    t->fd = open(
        cfg->serial_port,
        O_RDWR | O_NOCTTY | O_NONBLOCK
    );

    if (t->fd < 0) {
        free(t);
        return NULL;
    }

    if (tcgetattr(t->fd, &tio) != 0) {
        close(t->fd);
        free(t);
        return NULL;
    }

    cfmakeraw(&tio);

    if (cfsetispeed(&tio, speed) != 0 ||
        cfsetospeed(&tio, speed) != 0) {
        close(t->fd);
        free(t);
        return NULL;
    }

    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= (cfg->data_bits == 7) ? CS7 : CS8;

    if (cfg->parity == 'E') {
        tio.c_cflag |= PARENB;
        tio.c_cflag &= ~PARODD;
    } else if (cfg->parity == 'O') {
        tio.c_cflag |= PARENB;
        tio.c_cflag |= PARODD;
    } else {
        tio.c_cflag &= ~PARENB;
    }

    if (cfg->stop_bits == 2)
        tio.c_cflag |= CSTOPB;
    else
        tio.c_cflag &= ~CSTOPB;

    tio.c_cflag |= CLOCAL | CREAD;

#ifdef CRTSCTS
    tio.c_cflag &= ~CRTSCTS;
#endif

    if (tcsetattr(t->fd, TCSANOW, &tio) != 0) {
        close(t->fd);
        free(t);
        return NULL;
    }

    /* Switch back to blocking file descriptor; poll controls timeout. */
    {
        int flags = fcntl(t->fd, F_GETFL, 0);
        if (flags >= 0)
            fcntl(t->fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    tcflush(t->fd, TCIOFLUSH);

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

    if (t->fd >= 0)
        close(t->fd);

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
    int i;

    if (!t || t->fd < 0 || !dest ||
        address < 0 || address > 65535 ||
        count < 1 || count > 125) {
        if (t)
            snprintf(
                t->last_error,
                sizeof(t->last_error),
                "Invalid FC03 arguments"
            );
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

    tcflush(t->fd, TCIFLUSH);

    if (write_all(t, request, sizeof(request)) != 0)
        return -1;

    if (read_exact(t, header, sizeof(header)) != 0)
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
        uint8_t crc_tail[2];

        if (read_exact(t, crc_tail, sizeof(crc_tail)) != 0)
            return -1;

        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "Modbus exception: function 0x%02X code %u",
            (unsigned)header[1],
            (unsigned)header[2]
        );
        return -1;
    }

    if (header[1] != 0x03) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "Unexpected function 0x%02X",
            (unsigned)header[1]
        );
        return -1;
    }

    if (header[2] != (uint8_t)(count * 2)) {
        snprintf(
            t->last_error,
            sizeof(t->last_error),
            "Unexpected byte count %u (expected %u)",
            (unsigned)header[2],
            (unsigned)(count * 2)
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
            "CRC mismatch: calculated 0x%04X received 0x%04X",
            (unsigned)calculated_crc,
            (unsigned)received_crc
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
