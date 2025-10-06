SET(HEADERS
    Plugin.h
    ConnectionLine.h
)

SET(SOURCES
    Plugin.cpp
    ConnectionLine.cpp
)

find_package(fabrik)
if(NOT fabrik_FOUND)
    return()
endif()

get_target_property(FABRIK_CONFIG_DIR fabrik::fabrik LOCATION)
get_filename_component(FABRIK_CONFIG_DIR "${FABRIK_CONFIG_DIR}" DIRECTORY)
set(FABRIK_CONFIG_DIR "${FABRIK_CONFIG_DIR}/../../test/inputs")
set(FABRIK_CONFIG_FILES joints_position.txt joints_position_fixed.txt orientation.txt joints_constraint.txt constraint_type.txt bone_twist_constraints.txt)
if(EXISTS "${FABRIK_CONFIG_DIR}")
    foreach(FILE ${FABRIK_CONFIG_FILES})
        if(NOT EXISTS "${FABRIK_CONFIG_DIR}/${FILE}")
            set(FABRIK_CONFIG_DIR "")
        endif()
    endforeach()
else()
   set(FABRIK_CONFIG_DIR "")
endif()

message("Fabrik test directory: ${FABRIK_CONFIG_DIR}")
cover_add_plugin(FabrikFullBody ${HEADERS} ${SOURCES})

target_link_libraries(FabrikFullBody ${OPENSCENEGRAPH_LIBRARIES} fabrik::fabrik)

target_compile_definitions(FabrikFullBody PRIVATE FABRIK_CONFIG_DIR="${FABRIK_CONFIG_DIR}")