set(VRB_SOURCES
    PrintClientList.cpp
    Registry.cpp
    RegistryClass.cpp
    RegistryVariable.cpp
    RemoteClient.cpp
    SessionID.cpp	
    VrbSetUserInfoMessage.cpp
)

set(VRB_HEADERS
    PrintClientList.h
    Registry.h
    RegistryClass.h
    RegistryVariable.h
    RemoteClient.h
    SessionID.h
    VrbSetUserInfoMessage.h
)

ADD_COVISE_LIBRARY(coVRB ${VRB_SOURCES} ${VRB_HEADERS})
target_link_libraries(coVRB
                        coNet
                        coUtil 
)
COVISE_INSTALL_TARGET(coVRB)
COVISE_INSTALL_HEADERS(vrb ${VRB_HEADERS})

ADD_SUBDIRECTORY(client)
ADD_SUBDIRECTORY(server)

