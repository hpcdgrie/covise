
SET(HEADERS
Avatar.h
Bone.h
Plugin.h
ThreePointArm.cpp
)

SET(SOURCES
Avatar.cpp
Bone.cpp
Plugin.cpp
ThreePointArm.cpp
)

find_package(IK)
if(NOT ${IK_FOUND})
    return()
endif()

cover_add_plugin(Avatar ${HEADERS})
target_link_libraries(Avatar ${OPENSCENEGRAPH_LIBRARIES} ${IK_LIBRARIES})
target_include_directories(Avatar PRIVATE ${IK_INCLUDE_DIR})