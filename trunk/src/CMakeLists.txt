#User directory
SET(USER_DIRECTORY /home/azubiri)
#The application and its dependencies directory
SET(APP_DEP soft)
#CHAI3D directory
SET(CHAI3D chai3d)
#Version CHAI3D
SET(VERSION 3.1.1)

FIND_PACKAGE(GLUT REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

# driver source files
SET(SOURCES haptic_guides.cpp choose_filter.cpp openni_processor.cpp segmentation.cpp)

# application header files
SET(headers haptic_guides.h choose_filter.h openni_processor.h segmentation.h)

# locate the necessary dependencies
# add the necessary include directories
INCLUDE_DIRECTORIES(.)
INCLUDE_DIRECTORIES(OPENGL_INCLUDE_DIR)
#include directories from CHAI3D
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/src)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/Eigen)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/glew/include)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/giflib/include)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/lib3ds/include)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/libjpeg/include)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/libpng/include)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/openal/include)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/openal/OpenAL32/Include)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/openal/Alc)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/theoraplayer/include/theoraplayer)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/theoraplayer/external/ogg/include)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/theoraplayer/external/theora/include)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/theoraplayer/external/vorbis/include)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/theoraplayer/external/vorbis/lib)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/theoraplayer/src/Theora)
#include directories from PCL 1.8
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/build/include/)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/search/include)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/io/include)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/common/include/)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/kdtree/include/)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/features/include/)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/octree/include/)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/segmentation/include/)
INCLUDE_DIRECTORIES(/usr/include/ni)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/visualization/include/)
INCLUDE_DIRECTORIES(/usr/include/vtk-5.8/)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/geometry/include/)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/surface/include/)
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/filters/include/)
LINK_DIRECTORIES(${USER_DIRECTORY}/pcl-trunk/build/lib/)
SET(MYPCL -lpcl_kdtree -lpcl_search -lpcl_common -lboost_system -lboost_thread -lpcl_segmentation -lpcl_io -lpcl_surface -lpcl_filters -lpcl_features -lvtkCommon -lvtkFiltering -lOpenNI -lpcl_visualization -lvtkRendering)
#DHD dependency
INCLUDE_DIRECTORIES(${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/DHD/include)
SET(DHD_LIBRARY_DIRS ${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/DHD/lib/lin-x86_64)
LINK_DIRECTORIES(${DHD_LIBRARY_DIRS})

# static library source files
FILE(GLOB_RECURSE source_chai3d ${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/src/*.cpp)
# static library external dependencies source files
FILE(GLOB_RECURSE source_gif       ${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/giflib/src/*.c)
FILE(GLOB_RECURSE source_glew      ${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/glew/src/*.c)
FILE(GLOB_RECURSE source_3ds       ${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/lib3ds/src/*.c)
FILE(GLOB_RECURSE source_jpeg      ${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/libjpeg/src/*.c)
FILE(GLOB_RECURSE source_png       ${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/libpng/src/*.c)
FILE(GLOB_RECURSE source_openal    ${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/openal/*.c)
FILE(GLOB_RECURSE source_theorac   ${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/theoraplayer/*.c)
FILE(GLOB_RECURSE source_theoracpp ${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/theoraplayer/*.cpp)

FILE(GLOB_RECURSE source_theora_exclude ${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/theoraplayer/external/theora/lib/x86_vc/*.c)
LIST(REMOVE_ITEM source_theorac ${source_theora_exclude})

FILE(GLOB SOURCES "*.cpp")

# build flags
ADD_DEFINITIONS(${PROJECT_DEFINITIONS} -DGLEW_STATIC -DAL_ALEXT_PROTOTYPES -DAL_BUILD_LIBRARY -DAL_LIBTYPE_STATIC -D__THEORA -D_LIB)
# static library
ADD_LIBRARY(chai3d STATIC ${source_chai3d} ${source_gif} ${source_glew} ${source_3ds} ${source_jpeg} ${source_png} ${source_openal} ${source_theorac} ${source_theoracpp} ${source_dcmdata} ${source_libi2d} ${source_dcmimage} ${source_dcmimgle} ${source_oflog} ${source_ofstd})

# Chai3d library exports
SET(CHAI3D_INCLUDE_DIRS "${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/src" "${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/Eigen" "${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION}/external/glew/include" "${OPENGL_INCLUDE_DIR}")
SET(CHAI3D_LIBRARIES chai3d ${OPENGL_LIBRARIES})
SET(CHAI3D_DEFINITIONS ${PROJECT_DEFINITIONS})
SET(CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} drd)
SET(CHAI3D_LIBRARY_DIRS ${CHAI3D_LIBRARY_DIRS} ${DHD_LIBRARY_DIRS})
SET(CHAI3D_LIBRARIES ${CHAI3D_LIBRARIES} usb-1.0 rt pthread dl)
SET(CHAI3D_SOURCE_DIR ${USER_DIRECTORY}/${APP_DEP}/${CHAI3D}-${VERSION})


# create the shared library
ADD_LIBRARY(haptic_guides SHARED ${SOURCES})
SET(HAPTIC_GUIDES_LIBRARIES haptic_guides)

TARGET_LINK_LIBRARIES(haptic_guides ${OpenCV_LIBS} ${CHAI3D_LIBRARIES} ${GLUT_LIBRARIES} ${MYPCL})

# link necessary libraries
INSTALL(TARGETS haptic_guides
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib/iridrivers
        ARCHIVE DESTINATION lib/iridrivers)
INSTALL(FILES ${headers} DESTINATION include/iridrivers)
INSTALL(FILES ../Findhaptic_guides.cmake DESTINATION ${CMAKE_ROOT}/Modules/)
ADD_SUBDIRECTORY(examples)
