add_library(plugin_rgb INTERFACE)

target_sources(plugin_rgb INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/rgb.c
)

target_include_directories(plugin_rgb INTERFACE ${CMAKE_CURRENT_LIST_DIR})
