make_cuda_executable()

add_definitions(-DIMGUI_IMPL_OPENGL_LOADER_GLAD)

if(MKL_FOUND)
    add_definitions(-DEIGEN_USE_MKL_ALL)
endif()

include_directories(
    ${PLOG_INCLUDE_DIR}
    ${GLM_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR}
    ${GLAD_INCLUDE_DIR}
    ${IMGUI_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${TETGEN_INCLUDE_DIR}
    ${MKL_INCLUDE_DIR}
    ${CUDA_INCLUDE_DIRS}
    ${CUDA_TOOLKIT_ROOT_DIR}/samples/common/inc/
    ${Boost_INCLUDE_DIR}
    )

target_link_libraries(${PROJECT}
    ${OPENGL_LIBRARIES}
    ${GLFW_LIBRARIES}
    ${GLAD_LIBRARIES}
    ${IMGUI_LIBRARIES}
    ${OpenMP_CXX_LIBRARIES}
    ${TETGEN_LIBRARIES}
    ${MKL_LIBRARIES}
    "stdc++fs"
    ${CUDA_CUDART_LIBRARY}
    ${CUDA_CUBLAS_LIBRARIES}
    ${Boost_LIBRARIES} 
    )



