if(NOT OSGANIMATION_FOUND)
return()
endif()

cover_add_plugin(GhostAvatar GhostAvatar.cpp)
target_link_libraries(GhostAvatar ${OPENSCENEGRAPH_LIBRARIES})