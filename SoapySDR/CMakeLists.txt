cmake_minimum_required(VERSION 3.12)
project(SoapyMipiSDR CXX)

set(CMAKE_SYSTEM_IGNORE_PATH /usr/local /usr/local/lib /usr/local/include)

# 1. Performance Flags for Raspberry Pi 5 (Cortex-A76)
# -flto: Link Time Optimization (inlines filter code across translation units)
# -ffast-math: Allows ARM NEON to skip strict IEEE compliance for speed
add_compile_options(
    -O3 
    -mcpu=cortex-a76 
    -mtune=cortex-a76 
    -ftree-vectorize 
    -funsafe-math-optimizations 
    -ffast-math 
    -flto
)

find_package(PkgConfig REQUIRED)
set(ENV{PKG_CONFIG_PATH} "/usr/lib/aarch64-linux-gnu/pkgconfig:/usr/lib/pkgconfig:/usr/share/pkgconfig")

pkg_check_modules(SOAPY_SDR REQUIRED SoapySDR)

message(STATUS "SoapySDR Include: ${SOAPY_SDR_INCLUDE_DIRS}")
message(STATUS "SoapySDR Libs: ${SOAPY_SDR_LIBRARIES}")

add_library(SoapyMipiSDR MODULE
    Registration.cpp
    MipiDevice.cpp
    NEON.cpp
)

target_compile_features(SoapyMipiSDR PRIVATE cxx_std_17)
target_include_directories(SoapyMipiSDR PRIVATE ${SOAPY_SDR_INCLUDE_DIRS} .)

target_link_libraries(SoapyMipiSDR PRIVATE ${SOAPY_SDR_LIBRARIES})
# Link-Time Optimization requires linker flags as well
target_link_options(SoapyMipiSDR PRIVATE -flto ${FFTW3F_LDFLAGS})

set_target_properties(SoapyMipiSDR PROPERTIES 
    OUTPUT_NAME "mipi"
    BUILD_WITH_INSTALL_RPATH FALSE
    SKIP_BUILD_RPATH TRUE
)

execute_process(COMMAND ${PKG_CONFIG_EXECUTABLE} --variable=modulesdir SoapySDR
    OUTPUT_VARIABLE SOAPY_SDR_MODULE_PATH
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

if (NOT SOAPY_SDR_MODULE_PATH)
    #set(SOAPY_SDR_MODULE_PATH "/usr/lib/aarch64-linux-gnu/SoapySDR/modules0.8-3")
    set(SOAPY_SDR_MODULE_PATH "/usr/local/lib/SoapySDR/modules0.8-3")
endif()

message(STATUS "Installing to: ${SOAPY_SDR_MODULE_PATH}")
install(TARGETS SoapyMipiSDR DESTINATION ${SOAPY_SDR_MODULE_PATH})