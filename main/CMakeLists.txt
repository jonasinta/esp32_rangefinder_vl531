# Edit following two lines to set component requirements (see docs)
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )
set(COMPONENT_SRCS "main.c" "vl53l1_api.c" "vl53l1_api_calibration.c" "vl53l1_api_core.c" "vl53l1_api_debug.c" "vl53l1_api_preset_modes.c" "vl53l1_api_strings.c" "vl53l1_core.c" "vl53l1_core_support.c" "vl53l1_error_strings.c" "vl53l1_platform.c" "vl53l1_register_funcs.c" "vl53l1_silicon_core.c" "vl53l1_wait.c")
set(COMPONENT_ADD_INCLUDEDIRS "include")
register_component()
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-variable -USE_I2C_2V8k)

 #-USE_I2C_2V8k
