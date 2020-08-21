##############################################################################
# search paths
##############################################################################
SET(OPENPASS_INCLUDE_SEARCH_DIRS
  $ENV{EXTERNLIBS}/OpenPASS/include
  $ENV{COVISEDIR}/../simopenpass/OpenPass_Source_Code/OpenPASS
  ${GLOBAL_EXT_DIR}/OpenPASS/include
  ${GLOBAL_EXT_DIR}/include/OpenPASS
  ${OPENPASS_INCLUDE_SEARCH_DIR}
  /opt/OpenPASS/current
)

SET(OPENPASS_LIBRARY_SEARCH_DIRS
  $ENV{COVISEDIR}/../simopenpass/build/OpenPASS
  $ENV{EXTERNLIBS}/OpenPASS
  /opt/OpenPASS/current/lib/linux_x86
)

##############################################################################
# check for OpenPASS
##############################################################################
message(STATUS "-- checking for OpenPASS")

IF ( NOT OPENPASS_INCLUDE_DIRS )

    FOREACH(_SEARCH_DIR ${OPENPASS_INCLUDE_SEARCH_DIRS})
        FIND_PATH(_CUR_SEARCH
                NAMES Interfaces/agentInterface.h
                PATHS ${_SEARCH_DIR}
                NO_DEFAULT_PATH)
        IF (_CUR_SEARCH)
            LIST(APPEND _OPENPASS_FOUND_INC_DIRS ${_CUR_SEARCH})
        ENDIF(_CUR_SEARCH)
        SET(_CUR_SEARCH _CUR_SEARCH-NOTFOUND CACHE INTERNAL "internal use")
    ENDFOREACH(_SEARCH_DIR ${OPENPASS_INCLUDE_SEARCH_DIRS})


    FOREACH(_INC_DIR ${_OPENPASS_FOUND_INC_DIRS})
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/Interfaces)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/Common)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave/framework)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave/importer)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave/modelElements)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave/modelInterface)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave/observationInterface)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave/scheduler)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave/spawnPointInterface)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave/stochasticsInterface)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave/worldInterface)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave/manipulatorInterface)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave/eventDetectorInterface)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/OpenPassSlave/collisionDetectionInterface)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework/CoreShare)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR}/CoreFramework)
        LIST(APPEND _OPENPASS_INCLUDE_DIRS ${_INC_DIR})
    ENDFOREACH(_INC_DIR ${_BOOST_FOUND_INC_DIRS})

    IF (_OPENPASS_FOUND_INC_DIRS)
        SET(OPENPASS_INCLUDE_DIRS ${_OPENPASS_INCLUDE_DIRS} CACHE PATH "path to OPENPASS headers.")
    ENDIF (_OPENPASS_FOUND_INC_DIRS)

ENDIF ( NOT OPENPASS_INCLUDE_DIRS )

# release libraries
find_library(OPENPASS_COMMON_LIBRARY 
             NAMES Common
             PATHS ${OPENPASS_LIBRARY_SEARCH_DIRS}
             SUFFIXES bin
            )

find_library(OPENPASS_CORE_SLAVE_LIBRARY 
             NAMES CoreSlave
             PATHS ${OPENPASS_LIBRARY_SEARCH_DIRS}
             SUFFIXES bin
            )

find_library(OPENPASS_CORE_SHARED_LIBRARY 
             NAMES CoreShare
             PATHS ${OPENPASS_LIBRARY_SEARCH_DIRS}
             SUFFIXES bin
            )
			
# find debug libraries
find_library(OPENPASS_COMMON_LIBRARY_DEBUG
             NAMES Commond
             PATHS ${OPENPASS_LIBRARY_SEARCH_DIRS}
             SUFFIXES bin
            )

find_library(OPENPASS_CORE_SLAVE_LIBRARY_DEBUG
             NAMES CoreSlaved
             PATHS ${OPENPASS_LIBRARY_SEARCH_DIRS}
             SUFFIXES bin
            )

find_library(OPENPASS_CORE_SHARED_LIBRARY_DEBUG
             NAMES CoreShared
             PATHS ${OPENPASS_LIBRARY_SEARCH_DIRS}
             SUFFIXES bin
            )

IF(MSVC)
  
  IF(OPENPASS_COMMON_LIBRARY_DEBUG AND OPENPASS_COMMON_LIBRARY)
    SET(OPENPASS_LIBRARIES optimized ${OPENPASS_COMMON_LIBRARY} debug ${OPENPASS_COMMON_LIBRARY_DEBUG} optimized ${OPENPASS_CORE_SLAVE_LIBRARY} debug ${OPENPASS_CORE_SLAVE_LIBRARY_DEBUG} optimized ${OPENPASS_CORE_SHARED_LIBRARY} debug ${OPENPASS_CORE_SHARED_LIBRARY_DEBUG})
  ENDIF(OPENPASS_COMMON_LIBRARY_DEBUG AND OPENPASS_COMMON_LIBRARY)

  FIND_PACKAGE_HANDLE_STANDARD_ARGS(OPENPASS DEFAULT_MSG OPENPASS_COMMON_LIBRARY OPENPASS_COMMON_LIBRARY_DEBUG OPENPASS_CORE_SLAVE_LIBRARY OPENPASS_CORE_SLAVE_LIBRARY_DEBUG OPENPASS_CORE_SHARED_LIBRARY OPENPASS_CORE_SHARED_LIBRARY_DEBUG)

  MARK_AS_ADVANCED(OPENPASS_COMMON_LIBRARY OPENPASS_COMMON_LIBRARY_DEBUG OPENPASS_CORE_SLAVE_LIBRARY OPENPASS_CORE_SLAVE_LIBRARY_DEBUG OPENPASS_CORE_SHARED_LIBRARY OPENPASS_CORE_SHARED_LIBRARY_DEBUG)
  
ELSE(MSVC)
  # rest of the world
    SET(OPENPASS_LIBRARIES ${OPENPASS_COMMON_LIBRARY} ${OPENPASS_CORE_SLAVE_LIBRARY} ${OPENPASS_CORE_SHARED_LIBRARY})

  FIND_PACKAGE_HANDLE_STANDARD_ARGS(OPENPASS DEFAULT_MSG OPENPASS_COMMON_LIBRARY OPENPASS_CORE_SLAVE_LIBRARY OPENPASS_CORE_SHARED_LIBRARY)
  
  MARK_AS_ADVANCED(OPENPASS_COMMON_LIBRARY  OPENPASS_CORE_SLAVE_LIBRARY OPENPASS_CORE_SHARED_LIBRARY)
  
ENDIF(MSVC)