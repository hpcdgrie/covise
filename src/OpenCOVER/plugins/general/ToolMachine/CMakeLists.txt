
# using(OPEN62541)
using(VRML)

if(NOT open62541_FOUND)
     MESSAGE("open62541 not found, not building ToolMachine")
    return()
endif()


SET(HEADERS
Currents.h
Oct.h
Tool.h
ToolMachine.h
)

SET(SOURCES
Currents.cpp
Oct.cpp
Tool.cpp
ToolMachine.cpp
)

cover_add_plugin(ToolMachine)
target_link_libraries(ToolMachine coOpenOpcUaClient)

