cmake_minimum_required(VERSION 3.16)
project(cci_huawei_modbus_reader C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(WIN32)
    set(TRANSPORT src/modbus_transport_win32.c)
else()
    set(TRANSPORT src/modbus_transport_posix.c)
endif()

add_executable(
    cci_huawei_reader
    app/main.c
    drivers/huawei_sun2000.c
    ${TRANSPORT}
)

target_include_directories(
    cci_huawei_reader
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/drivers
)

if(MSVC)
    target_compile_options(cci_huawei_reader PRIVATE /W4)
else()
    target_compile_options(
        cci_huawei_reader
        PRIVATE -Wall -Wextra -Wpedantic
    )
endif()
