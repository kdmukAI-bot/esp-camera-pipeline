idf_component_register(
    SRCS
        "src/cam_pipeline_qr.c"
    INCLUDE_DIRS "include"
    REQUIRES k_quirc esp-camera-pipeline freertos esp_timer
)
