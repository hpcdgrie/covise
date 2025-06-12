if (NOT COVISE_USE_QT)
    return()
endif()

covise_find_package(nlohmann_json 3.10.0)

if (NOT nlohmann_json_FOUND)
    #message("nlohmann_json not found")
    return()
endif()

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

if(CMAKE_VERSION VERSION_LESS "3.7.0")
    set(CMAKE_INCLUDE_CURRENT_DIR ON)
endif()
    

set(SOURCES
    main.cpp
    flowlayout.cpp
)

set(HEADERS
    flowlayout.h
)

SET(VRB_MOC_SRCS
  
  
)

ADD_COVISE_EXECUTABLE(hlrsDemo ${VRB_MOC_SRCS})

target_link_libraries(hlrsDemo nlohmann_json::nlohmann_json) 
qt_use_modules(hlrsDemo Core Gui Widgets)

COVISE_INSTALL_TARGET(hlrsDemo)
