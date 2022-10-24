CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# CMAKE_MODULE_PATH, 设置 Module 文件查找路径 
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

# arm_linux_setup 为自定义的 Module 文件
# 用于 Arm v7 的交叉编译
include(arm_linux_setup)

# 设置项目名称
project(10MA_SC)

# 设置生成的目标文件的名称
set(TARGET_NAME 10MA_SC_Module.elf)

# 设置项目相关源码文件
set(SC_SRC 
        src/geometry_data.cpp;
        src/global_structure.cpp;
        src/global_var_func.cpp;
        src/inifile.cpp;
        src/main.cpp;
        src/trace.cpp;
        src/variable.cpp;
        src/alarm_processor/alarm_processor.cpp;
        src/Algorithm/coord_processor.cpp;
        src/spindle/spindle_control.cpp
)

set(CHANNEL_SRC
        src/channel/channel_control.cpp;
        src/channel/channel_engine.cpp;
        src/channel/channel_mode_group.cpp;
)

set(COMMUNICATION_SRC
        src/communication/ad_communication.cpp;
        src/communication/hmi_communication.cpp;
        src/communication/mc_communication.cpp;
        src/communication/mc_communication_arm.cpp;
        src/communication/mi_communication.cpp;
)

set(COMPILER_SRC
        src/compiler/compile_message.cpp;
        src/compiler/compiler.cpp;
        src/compiler/compiler_data.cpp;
        src/compiler/lexer.cpp;
        src/compiler/parser.cpp;
)

set(PARAMETER_SRC
        src/parameter/parm_manager.cpp;
)

set(PMC_SRC
        src/pmc/pmc_axis_ctrl.cpp;
        src/pmc/pmc_register.cpp;
)

set(TOOL_COMPENSATE_SRC
        src/tool_compensate/tool_comp_data.cpp;
        src/tool_compensate/tool_compensate.cpp;
)

set (SC_SRC ${SC_SRC} ${COMMUNICATION_SRC} ${COMPILER_SRC} ${CHANNEL_SRC}
			${PARAMETER_SRC} ${PMC_SRC} ${TOOL_COMPENSATE_SRC})

# 生成名称为 ${TARGET_NAME} 的可执行文件
add_executable(${TARGET_NAME} ${SC_SRC})

# 设置头文件的查找路径
target_include_directories(${TARGET_NAME} PRIVATE inc)
target_include_directories(${TARGET_NAME} PRIVATE inc/alarm_processor)
target_include_directories(${TARGET_NAME} PRIVATE inc/Algorithm)
target_include_directories(${TARGET_NAME} PRIVATE inc/channel)
target_include_directories(${TARGET_NAME} PRIVATE inc/communication)
target_include_directories(${TARGET_NAME} PRIVATE inc/compiler)
target_include_directories(${TARGET_NAME} PRIVATE inc/parameter)
target_include_directories(${TARGET_NAME} PRIVATE inc/pmc)
target_include_directories(${TARGET_NAME} PRIVATE inc/tool_compensate)
target_include_directories(${TARGET_NAME} PRIVATE inc/spindle)
target_include_directories(${TARGET_NAME} PRIVATE lib/inc)

# 设置库文件的查找路径
link_directories(${CMAKE_SOURCE_DIR}/lib/release)

# 导入静态链接库
add_library(LibLicense STATIC IMPORTED)
set_property(TARGET LibLicense PROPERTY IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/release/libLitronucLicense.a)
add_library(LibMosquitpp STATIC IMPORTED)
set_property(TARGET LibMosquitpp PROPERTY IMPORTED_LOCATION ${CMAKE_SOURCE_DIR}/lib/release/libmosquittopp_static.a)

# 链接库文件
target_link_libraries(${TARGET_NAME} PRIVATE pthread LibLicense LibMosquitpp)

# 设置编译选项
target_compile_options(${TARGET_NAME} PRIVATE
                            -Wall
                            -O2
                            -Wl,-Map,project.map
                            -fmessage-length=0
                            -MT"$@")










