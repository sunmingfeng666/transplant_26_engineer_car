cmake_minimum_required(VERSION 3.22)

# 项目基本配置
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
set(CMAKE_PROJECT_NAME H7_Framework)
project(${CMAKE_PROJECT_NAME} C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 硬件架构配置
set(CPU_FLAGS -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)

file(GLOB_RECURSE ALL_USER_SOURCES CONFIGURE_DEPENDS "User/*.c" "User/*.cpp" "User/*.s")
set(USER_SOURCES ${ALL_USER_SOURCES})
list(FILTER USER_SOURCES EXCLUDE REGEX "/User/App/")

# 自动递归包含头文件，并剔除 App 目录路径
file(GLOB_RECURSE ALL_H_FILES CONFIGURE_DEPENDS "User/*.h" "User/*.hpp" "Core/*.h" )
set(ALL_INCLUDE_DIRS "")
foreach(FILE_PATH ${ALL_H_FILES})
    get_filename_component(DIR_PATH ${FILE_PATH} DIRECTORY)
    list(APPEND ALL_INCLUDE_DIRS ${DIR_PATH})
endforeach()
list(REMOVE_DUPLICATES ALL_INCLUDE_DIRS)
list(FILTER ALL_INCLUDE_DIRS EXCLUDE REGEX "/User/App/")

file(GLOB_RECURSE CMSIS_DSP_SOURCES
        "Middlewares/Third_Party/CMSIS-DSP/Source/*/*.c"
)

set(CMSIS_DSP_MAIN_SOURCES "")
foreach(SOURCE_FILE ${CMSIS_DSP_SOURCES})
    # 获取文件名和所在目录名
    get_filename_component(FILE_NAME ${SOURCE_FILE} NAME)
    get_filename_component(DIR_NAME ${SOURCE_FILE} DIRECTORY)
    get_filename_component(DIR_NAME ${DIR_NAME} NAME)

    if(FILE_NAME STREQUAL "${DIR_NAME}.c")
        list(APPEND CMSIS_DSP_MAIN_SOURCES ${SOURCE_FILE})
    endif()
endforeach()

set(CMSIS_DSP_SOURCES ${CMSIS_DSP_MAIN_SOURCES})

add_executable(${CMAKE_PROJECT_NAME} ${USER_SOURCES} ${CMSIS_DSP_SOURCES})

target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
        ${ALL_INCLUDE_DIRS}
        Core/Inc
        Drivers/CMSIS/Include
        Middlewares/Third_Party/CMSIS-DSP/Include
        Middlewares/Third_Party/CMSIS-DSP/PrivateInclude
        Drivers/CMSIS/Device/ST/STM32H7xx/Include
        Drivers/STM32H7xx_HAL_Driver/Inc
)

# 编译选项与宏定义
target_compile_definitions(${CMAKE_PROJECT_NAME} PUBLIC
        STM32H723xx USE_HAL_DRIVER ARM_MATH_CM7 __FPU_PRESENT=1U
        HAL_ASSERT_DISABLE=1 ARM_MATH_MATRIX_CHECK=0 ARM_MATH_ROUNDING=0
        $<$<CONFIG:Debug>:DEBUG_MODE> $<$<CONFIG:Release>:RELEASE_MODE>
)

target_compile_options(${CMAKE_PROJECT_NAME} PUBLIC
        ${CPU_FLAGS}
        -Wall -Wextra -Wno-comment -Wno-unused-parameter -Wno-unused-but-set-variable
        -Werror=implicit-function-declaration -Werror=incompatible-pointer-types
        -fdata-sections -ffunction-sections
        $<$<CONFIG:Debug>:-Og -g3 -fno-omit-frame-pointer -fno-lto>
        $<$<CONFIG:Release>:-O3 -g3 -funroll-loops>
)

# 链接配置
target_link_options(${CMAKE_PROJECT_NAME} PUBLIC
        ${CPU_FLAGS}
        -Wl,--gc-sections
        -Wl,--print-memory-usage
        -Wl,--relax
        $<$<CONFIG:Release>:-flto=auto>
)

#set(ACTIVE_APP "1_Catapult_Hero")
set(ACTIVE_APP "2_Engineer")

add_subdirectory(cmake/stm32cubemx)
add_subdirectory(User/App/${ACTIVE_APP})

target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx
        m
)

# 后处理
set(HEX_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex)
set(BIN_FILE ${PROJECT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin)

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${HEX_FILE}"
        COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${CMAKE_PROJECT_NAME}> "${BIN_FILE}"
        COMMENT "Generating Output Files..."
)
