
SET(HEADERS
Bone.h
Avatar.h
Plugin.h
)

SET(SOURCES
Bone.cpp
Avatar.cpp
Plugin.cpp
)

find_package(fabrik)
if(NOT ${fabrik_FOUND})
    return()
endif()


cover_add_plugin(Avatar ${HEADERS} ${RTIK_HEADERS} )
target_link_libraries(Avatar ${OPENSCENEGRAPH_LIBRARIES} fabrik::fabrik)
