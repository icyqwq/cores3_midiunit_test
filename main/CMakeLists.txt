file(GLOB EXTERN_FILES resources/*)


idf_component_register(
    SRC_DIRS
        "."

    INCLUDE_DIRS
        "."

    EMBED_FILES ${EXTERN_FILES}
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format" "-Wno-error=narrowing")
target_compile_definitions(${COMPONENT_LIB} PUBLIC "-DLOG_LOCAL_LEVEL=ESP_LOG_VERBOSE")