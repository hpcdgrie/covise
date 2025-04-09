
SET(HEADERS
Plugin.h
)

SET(SOURCES
Plugin.cpp
)

find_package(fabrik)
if(NOT ${fabrik_FOUND})
    return()
endif()


cover_add_plugin(FabrikFullBody ${HEADERS} ${SOURCES})
target_link_libraries(FabrikFullBody ${OPENSCENEGRAPH_LIBRARIES} fabrik::fabrik)
