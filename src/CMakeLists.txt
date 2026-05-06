add_library(embdC_quat STATIC embdC_quat.c)

target_include_directories(embdC_quat PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(embdC_quat PRIVATE m)
