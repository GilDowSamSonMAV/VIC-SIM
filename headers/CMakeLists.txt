# Make headers available as an include directory to the rest of the project
add_library(sim_headers INTERFACE)

target_include_directories(sim_headers
    INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}
)
