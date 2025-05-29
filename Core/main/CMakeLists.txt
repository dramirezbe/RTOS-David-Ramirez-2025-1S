file(GLOB LIB_SOURCES "${CMAKE_CURRENT_LIST_DIR}/lib/*.c")

idf_component_register(
    SRCS
        "main.c"
        ${LIB_SOURCES}
    INCLUDE_DIRS
        "."
        "lib"
)
