idf_component_register(
    SRCS
        "src/k_quirc.c"
        "src/k_quirc_version.c"
        "src/k_quirc_identify.c"
        "src/k_quirc_decode.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
)

# Feature flags driven by Kconfig (replaces hardcoded compile definitions)
if(CONFIG_K_QUIRC_ADAPTIVE_THRESHOLD)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC K_QUIRC_ADAPTIVE_THRESHOLD)
endif()

if(CONFIG_K_QUIRC_BILINEAR_THRESHOLD)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC K_QUIRC_BILINEAR_THRESHOLD)
endif()

if(CONFIG_K_QUIRC_DEBUG)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC K_QUIRC_DEBUG)
endif()
