cmake_minimum_required(VERSION 3.20)

add_subdirectory("SDL")
add_subdirectory("librevfree")
add_subdirectory("json")

set(IMGUI_DIR "${CMAKE_CURRENT_SOURCE_DIR}/imgui")
file(GLOB IMGUI_SOURCE CONFIGURE_DEPENDS
    "${IMGUI_DIR}/*.cpp"
    "${IMGUI_DIR}/*.h")

set(IMGUI_BACKENDS sdl3 sdlrenderer3)
foreach(BACKEND ${IMGUI_BACKENDS})
    file(GLOB BACKEND_SOURCE CONFIGURE_DEPENDS
        "${IMGUI_DIR}/backends/imgui_impl_${BACKEND}.*")

    list(APPEND IMGUI_SOURCE ${BACKEND_SOURCE})
endforeach()

add_library(imgui STATIC ${IMGUI_SOURCE})
target_link_libraries(imgui PUBLIC SDL3-static)
target_include_directories(imgui PUBLIC ${IMGUI_DIR})
set_target_properties(imgui PROPERTIES CXX_STANDARD 20)