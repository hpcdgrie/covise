# @file
# 
# CMakeLists.txt for sys controller
#
# @author Blasius Czink

IF(COVISE_BUILD_WEBSERVICE)
  ADD_DEFINITIONS(-DHAVE_GSOAP)
ENDIF()

SET(CONTROLLER_SOURCES
  config.cpp #
  exception.cpp #
  util.cpp #
  netLink.cpp #
  object.cpp #list
  port.cpp #object list
  process.cpp #port
  userinterface.cpp #process
  controlProcess.cpp #config
  module.cpp #process, netlink
  crb.cpp #process
  host.cpp #process config
  renderModule.cpp #module ui
  modui.cpp #ui, module, process
  global.cpp #controlProcess, object, modui
  handler.cpp #host, ui, config
  main.cpp #
)

SET(CONTROLLER_HEADERS
  config.h #
  exception.h #
  list.h #
  util.h #
  netLink.h #
  object.h #list
  port.h #object list
  process.h #port
  userinterface.h #process
  controlProcess.h #config
  module.h #process, netlink
  crb.h #process
  host.h #process config
  renderModule.h #module ui
  modui.h #ui, module, process
  global.h #controlProcess, object, modui
  handler.h #host, ui, config

)

ADD_COVISE_EXECUTABLE(covise ${CONTROLLER_SOURCES} ${CONTROLLER_HEADERS})
TARGET_LINK_LIBRARIES(covise coAppl coApi coCore coConfig coUtil coVRBClient coMessages) 
qt_use_modules(covise Core)

COVISE_INSTALL_TARGET(covise)
