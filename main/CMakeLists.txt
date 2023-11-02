# file(GLOB EXTERN_FILES ../assets/binary/*)



idf_component_register(
    SRC_DIRS
        "."

    INCLUDE_DIRS
        "."

    EMBED_FILES ${EXTERN_FILES}
)

target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format" "-Wno-error=narrowing")