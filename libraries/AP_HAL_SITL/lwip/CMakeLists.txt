cmake_minimum_required(VERSION 3.8)

project(lwip C)

set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(LWIP_CONTRIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lwip-contrib)
set(LWIP_DIR         ${CMAKE_CURRENT_SOURCE_DIR}/../../../modules/lwip)

include(${LWIP_CONTRIB_DIR}/ports/CMakeCommon.cmake)

set (LWIP_DEFINITIONS -DLWIP_DEBUG)
set (LWIP_INCLUDE_DIRS
    "${LWIP_DIR}/src/include"
    "${LWIP_CONTRIB_DIR}/"
    "${LWIP_CONTRIB_DIR}/ports/unix/port/include"
    "."
)

include(${LWIP_DIR}/src/Filelists.cmake)
include(${LWIP_CONTRIB_DIR}/Filelists.cmake)
include(${LWIP_CONTRIB_DIR}/ports/unix/Filelists.cmake)

add_library(lwip STATIC default_netif.c)
target_include_directories(lwip PRIVATE ${LWIP_INCLUDE_DIRS})
target_compile_options(lwip PRIVATE ${LWIP_COMPILER_FLAGS})
target_compile_definitions(lwip PRIVATE ${LWIP_DEFINITIONS} ${LWIP_MBEDTLS_DEFINITIONS})
target_link_libraries(lwip ${LWIP_SANITIZER_LIBS} lwipallapps lwipcontribportunix lwipcore)
