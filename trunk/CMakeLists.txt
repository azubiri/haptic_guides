# Pre-requisites about cmake itself
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

if(COMMAND cmake_policy)
  cmake_policy(SET CMP0005 NEW) 
  cmake_policy(SET CMP0003 NEW)
   
endif(COMMAND cmake_policy)

# The project name and the type of project
PROJECT(haptic-guides)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)
SET(CMAKE_INSTALL_PREFIX /usr/local)

IF (NOT CMAKE_BUILD_TYPE)
 SET(CMAKE_BUILD_TYPE "DEBUG") 
ENDIF (NOT CMAKE_BUILD_TYPE)

#SET(CMAKE_CXX_FLAGS_DEBUG "-g -Wall -D_REENTRANT")
#SET(CMAKE_CXX_FLAGS_RELEASE "-O3 -D_REENTRANT")

add_definitions (-DLINUX)
SET(CMAKE_CXX_FLAGS                "${CMAKE_CXX_FLAGS}                -fPIC -march=native -Wno-deprecated -std=c++0x")
SET(CMAKE_CXX_FLAGS_DEBUG          "${CMAKE_CXX_FLAGS_DEBUG}          -fPIC -march=native -Wno-deprecated -std=c++0x")
SET(CMAKE_CXX_FLAGS_MINSIZEREL     "${CMAKE_CXX_FLAGS_MINSIZEREL}     -fPIC -march=native -Wno-deprecated -std=c++0x")
SET(CMAKE_CXX_FLAGS_RELEASE        "${CMAKE_CXX_FLAGS_RELEASE}        -fPIC -march=native -Wno-deprecated -std=c++0x")
SET(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} -fPIC -march=native -Wno-deprecated -std=c++0x")
SET(CMAKE_C_FLAGS                  "${CMAKE_C_FLAGS}                  -fPIC -march=native -Wno-deprecated")
SET(CMAKE_C_FLAGS_DEBUG            "${CMAKE_C_FLAGS_DEBUG}            -fPIC -march=native -Wno-deprecated")
SET(CMAKE_C_FLAGS_MINSIZEREL       "${CMAKE_C_FLAGS_MINSIZEREL}       -fPIC -march=native -Wno-deprecated")
SET(CMAKE_C_FLAGS_RELEASE          "${CMAKE_C_FLAGS_RELEASE}          -fPIC -march=native -Wno-deprecated")
SET(CMAKE_C_FLAGS_RELWITHDEBINFO   "${CMAKE_C_FLAGS_RELWITHDEBINFO}   -fPIC -march=native -Wno-deprecated")

ADD_SUBDIRECTORY(src)

FIND_PACKAGE(Doxygen)

FIND_PATH(IRI_DOC_DIR doxygen.conf ${CMAKE_SOURCE_DIR}/doc/iri_doc/)
IF (IRI_DOC_DIR)
  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/iri_doc/doxygen.conf)
ELSE (IRI_DOC_DIR)
  ADD_CUSTOM_TARGET (doc ${DOXYGEN_EXECUTABLE} ${CMAKE_SOURCE_DIR}/doc/doxygen.conf)
ENDIF (IRI_DOC_DIR)

ADD_CUSTOM_TARGET (distclean @echo cleaning cmake files)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distribution clean"
    COMMAND make ARGS clean
    COMMAND rm ARGS -rf ${CMAKE_SOURCE_DIR}/build/*

    TARGET  distclean
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "distclean only implemented in unix"
    TARGET  distclean
  )
ENDIF(UNIX)

ADD_CUSTOM_TARGET (uninstall @echo uninstall package)

IF (UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall package"
    COMMAND xargs ARGS rm < install_manifest.txt

    TARGET  uninstall
  )
ELSE(UNIX)
  ADD_CUSTOM_COMMAND(
    COMMENT "uninstall only implemented in unix"
    TARGET  uninstall
  )
ENDIF(UNIX)


