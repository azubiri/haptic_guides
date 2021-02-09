#edit the following line to add the librarie's header files
FIND_PATH(haptic_guides_INCLUDE_DIR haptic_guides.h /usr/include/iridrivers /usr/local/include/iridrivers)

FIND_LIBRARY(haptic_guides_LIBRARY
    NAMES haptic_guides
    PATHS /usr/lib /usr/local/lib /usr/local/lib/iridrivers) 

IF (haptic_guides_INCLUDE_DIR AND haptic_guides_LIBRARY)
   SET(haptic_guides_FOUND TRUE)
ENDIF (haptic_guides_INCLUDE_DIR AND haptic_guides_LIBRARY)

IF (haptic_guides_FOUND)
   IF (NOT haptic_guides_FIND_QUIETLY)
      MESSAGE(STATUS "Found haptic_guides: ${haptic_guides_LIBRARY}")
   ENDIF (NOT haptic_guides_FIND_QUIETLY)
ELSE (haptic_guides_FOUND)
   IF (haptic_guides_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find haptic_guides")
   ENDIF (haptic_guides_FIND_REQUIRED)
ENDIF (haptic_guides_FOUND)

