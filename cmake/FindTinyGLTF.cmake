# - Find TinyGLTF
# Find the TinyGLTF includes and library
#
#  TinyGLTF_INCLUDE_DIR - Where to find TinyGLTF includes
#  TinyGLTF_FOUND       - True if TinyGLTF was found

FIND_PATH(TinyGLTF_INCLUDE_DIR "tiny_gltf.h"
  PATHS
  $ENV{TinyGLTF_HOME}/include
  $ENV{EXTERNLIBS}/TinyGLTF/include
  PATH_SUFFIXES tinygltf
  DOC "TinyGLTF - Headers"
)

SET(TinyGLTF_NAMES TinyGLTF tinygltf)
SET(TinyGLTF_DBG_NAMES TinyGLTFd)

FIND_LIBRARY(TinyGLTF_LIBRARY NAMES ${TinyGLTF_NAMES}
  PATHS
  $ENV{TinyGLTF_HOME}
  $ENV{EXTERNLIBS}/TinyGLTF
  PATH_SUFFIXES lib lib64
  DOC "TinyGLTF - Library"
)

INCLUDE(FindPackageHandleStandardArgs)

FIND_PACKAGE_HANDLE_STANDARD_ARGS(TinyGLTF DEFAULT_MSG TinyGLTF_INCLUDE_DIR)

MARK_AS_ADVANCED(TinyGLTF_INCLUDE_DIR)

IF(TinyGLTF_FOUND)
  SET(TinyGLTF_INCLUDE_DIRS ${TinyGLTF_INCLUDE_DIR})
ENDIF(TinyGLTF_FOUND)
