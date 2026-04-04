idf_component_register(
    SRCS
        "src/esp_cam_pipeline.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES freertos esp_timer
    PRIV_REQUIRES esp_hw_support
)

# PPA support (P4 only) — detected automatically via SOC capability
if(CONFIG_SOC_PPA_SUPPORTED)
    idf_component_get_property(ppa_lib esp_driver_ppa COMPONENT_LIB)
    target_link_libraries(${COMPONENT_LIB} PRIVATE ${ppa_lib})
endif()
