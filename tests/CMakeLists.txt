

set(VRB_TEST_SOURCES 
    CRB_EXEC.cpp
    coSpawnProgram.cpp
    connection.cpp
    main.cpp
    messageExchange.cpp
    message_macros.cpp
    testObject.cpp
    tryPrint.cpp
    syncVar.cpp
)

set(VRB_TEST_HEADER 
    CRB_EXEC.h
    coSpawnProgram.h
    connection.h
    messageExchange.h
    message_macros.h
    testObject.h
    tryPrint.h
    syncVar.h
)

ADD_COVISE_EXECUTABLE(VrbTests ${VRB_TEST_SOURCES} ${VRB_TEST_HEADER})
target_link_libraries(VrbTests coVRBClient coMessages)
target_include_directories(VrbTests PRIVATE 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../src/kernel>
)

ADD_DEFINITIONS(-DHAVE_COVISE)

