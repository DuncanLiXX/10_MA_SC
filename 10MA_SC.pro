INCLUDEPATH += inc \
    inc/Algorithm \
    inc/alarm_processor \
    inc/channel \
    inc/communication \
    inc/compiler \
    inc/parameter \
    inc/pmc \
    inc/tool_compensate \


HEADERS += \
    inc/Algorithm/coord_processor.h \
    inc/alarm_processor/alarm_processor.h \
    inc/channel/channel_control.h \
    inc/channel/channel_data.h \
    inc/channel/channel_engine.h \
    inc/channel/channel_mode_group.h \
    inc/circular_buffer.h \
    inc/communication/ad_communication.h \
    inc/communication/comm_data_definition.h \
    inc/communication/hmi_communication.h \
    inc/communication/mc_communication.h \
    inc/communication/mc_communication_arm.h \
    inc/communication/mi_communication.h \
    inc/compiler/compile_message.h \
    inc/compiler/compiler.h \
    inc/compiler/compiler_data.h \
    inc/compiler/lexer.h \
    inc/compiler/parser.h \
    inc/data_stack.h \
    inc/geometry_data.h \
    inc/global_definition.h \
    inc/global_include.h \
    inc/global_structure.h \
    inc/hmi_shared_data.h \
    inc/inifile.h \
    inc/list_buffer.h \
    inc/parameter/parm_definition.h \
    inc/parameter/parm_manager.h \
    inc/pmc/pmc_axis_ctrl.h \
    inc/pmc/pmc_register.h \
    inc/tool_compensate/tool_comp_data.h \
    inc/tool_compensate/tool_compensate.h \
    inc/tool_compensate/tools_comp.h \
    inc/trace.h \
    inc/variable.h \
    src/license_interface.h

DISTFILES += \
    src/README.txt

SOURCES += \
    src/Algorithm/coord_processor.cpp \
    src/alarm_processor/alarm_processor.cpp \
    src/channel/channel_control.cpp \
    src/channel/channel_engine.cpp \
    src/channel/channel_mode_group.cpp \
    src/communication/ad_communication.cpp \
    src/communication/hmi_communication.cpp \
    src/communication/mc_communication.cpp \
    src/communication/mc_communication_arm.cpp \
    src/communication/mi_communication.cpp \
    src/compiler/compile_message.cpp \
    src/compiler/compiler.cpp \
    src/compiler/compiler_data.cpp \
    src/compiler/lexer.cpp \
    src/compiler/parser.cpp \
    src/geometry_data.cpp \
    src/global_structure.cpp \
    src/global_var_func.cpp \
    src/inifile.cpp \
    src/main.cpp \
    src/parameter/parm_manager.cpp \
    src/pmc/pmc_axis_ctrl.cpp \
    src/pmc/pmc_register.cpp \
    src/tool_compensate/tool_comp_data.cpp \
    src/tool_compensate/tool_compensate.cpp \
    src/tool_compensate/tool_math.cpp \
    src/tool_compensate/tools_comp.cpp \
    src/trace.cpp \
    src/variable.cpp
