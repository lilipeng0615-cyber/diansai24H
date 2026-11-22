# ARM Cortex-M 交叉编译工具链（TI 发行的 GNU Arm GCC 9.2.1）
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR cortex-m0plus)

set(TOOLCHAIN_ROOT "C:/ti/gcc_arm_none_eabi_9_2_1")
set(CMAKE_C_COMPILER   "${TOOLCHAIN_ROOT}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_ASM_COMPILER "${TOOLCHAIN_ROOT}/bin/arm-none-eabi-gcc.exe")
set(CMAKE_OBJCOPY      "${TOOLCHAIN_ROOT}/bin/arm-none-eabi-objcopy.exe")
set(CMAKE_SIZE         "${TOOLCHAIN_ROOT}/bin/arm-none-eabi-size.exe")

# 汇编语言也要带上目标架构，否则 .s/.S 会按宿主默认架构汇编
set(CMAKE_ASM_FLAGS "-mcpu=cortex-m0plus -mthumb")

set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
