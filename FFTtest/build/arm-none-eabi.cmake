# arm-none-eabi.cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Specify the cross compiler
set(CMAKE_C_COMPILER "D:/CMSIS-DSP/arm-none/arm-gnu-toolchain-14.2.rel1-mingw-w64-i686-arm-none-eabi/bin/arm-none-eabi-gcc.exe")
set(CMAKE_CXX_COMPILER "D:/CMSIS-DSP/arm-none/arm-gnu-toolchain-14.2.rel1-mingw-w64-i686-arm-none-eabi/bin/arm-none-eabi-g++.exe")

# Set additional flags with hardware floating-point support
set(CMAKE_C_FLAGS "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nosys.specs")
set(CMAKE_EXE_LINKER_FLAGS "-mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 --specs=nosys.specs")

# Don't use the default linker
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
