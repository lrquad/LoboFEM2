add_subdirectory(${CMAKE_SOURCE_DIR}/external/tetgen)
set_target_properties(tetgen PROPERTIES FOLDER external)
set(TETGEN_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/external/tetgen)
set(TETGEN_LIBRARIES ${TETGEN_LIBRARIES} tetgen)