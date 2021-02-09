#FILE(GLOB example "haptic_guides_test.cpp")
# create an example application
ADD_EXECUTABLE(haptic_guides_test haptic_guides_test.cpp)
# link necessary libraries
#MESSAGE(${OpenCV_LIBS} "asdf")
TARGET_LINK_LIBRARIES(haptic_guides_test ${HAPTIC_GUIDES_LIBRARIES})
#add_executable( DisplayImage DisplayImage.cpp )
#TARGET_LINK_LIBRARIES(DisplayImage ${OpenCV_LIBS} ${HAPTIC_GUIDES_LIBRARIES} ${CHAI3D_LIBRARIES} ${GLUT_LIBRARIES} ${MYPCL} )
