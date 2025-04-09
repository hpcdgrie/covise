
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

find_package(IK)
if(NOT ${IK_FOUND})
    return()
endif()

cover_add_plugin(AvatarCometIK ${HEADERS})
target_link_libraries(AvatarCometIK ${OPENSCENEGRAPH_LIBRARIES} ${IK_LIBRARIES})
target_include_directories(AvatarCometIK PRIVATE ${IK_INCLUDE_DIR})