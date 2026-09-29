# CCI Huawei SUN2000 Modbus RTU reader — PC + OpenWrt

This version has **no libmodbus dependency**.

The same Huawei driver is used on both targets:

```text
drivers/huawei_sun2000.c
```

Only the serial transport changes at compile time:

```text
Windows PC   -> src/modbus_transport_win32.c
OpenWrt      -> src/modbus_transport_posix.c
```

The OpenWrt transport uses only normal POSIX/Linux facilities:

- open()
- termios
- poll()
- read()
- write()
- tcdrain()
- CRC16 implemented locally

So it is suitable for a small embedded OpenWrt target using musl libc.

## Windows test

Simulator:

```powershell
python sun2000_100ktl_m2_rtu_sim_v4.py --port COM1
```

Build:

```powershell
cmake -S . -B build
cmake --build build --config Release
```

Run:

```powershell
.\build\Release\cci_huawei_reader.exe COM2 9600 1
```

## OpenWrt

Typical runtime serial ports:

```text
/dev/ttyS1
/dev/ttyUSB0
/dev/ttyACM0
```

If compiling directly on a Linux/OpenWrt system with a compiler:

```sh
make -f Makefile.openwrt
```

Run:

```sh
./cci_huawei_reader /dev/ttyUSB0 9600 1
```

For a real embedded target, normally cross-compile with the OpenWrt SDK/toolchain:

```sh
make -f Makefile.openwrt \
    CC=<openwrt-toolchain-prefix>-gcc
```

The executable has no extra Modbus library runtime dependency.

## Architecture

```text
Huawei SUN2000-100KTL-M2
         |
       RS485
         |
     Modbus RTU
         |
 modbus_transport.h
     /         \
Windows       POSIX/OpenWrt
     \         /
 huawei_sun2000.c
         |
cci_inverter_measurements_t
         |
 IEC 61850 layer later
```

## Important RS485 note

If OpenWrt sees a USB-to-RS485 adapter as `/dev/ttyUSB0`, direction control is
usually handled by the adapter.

If the MediaTek board exposes a native UART connected to an external RS485
transceiver, DE/RE direction control may require kernel RS485 mode or a GPIO.
That hardware-specific piece belongs in the transport layer and does not change
the Huawei driver.
