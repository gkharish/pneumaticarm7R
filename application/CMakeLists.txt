# provide path to dependency libraries

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
#INCLUDE_DIRECTORIES(${XENOMAI_RTDM_INCLUDE_DIR})

LINK_DIRECTORIES(${${PROJECT_NAME}_BINARY_DIR}/src)
LINK_DIRECTORIES(/usr/xenomai/lib)

ADD_EXECUTABLE(test-pneumaticarm7R test_pneumaticarm7R.cpp)
TARGET_LINK_LIBRARIES(test-pneumaticarm7R pneumaticarm7R)
PKG_CONFIG_USE_DEPENDENCY (test-pneumaticarm7R libxenomai_rtdm)
PKG_CONFIG_USE_DEPENDENCY (test-pneumaticarm7R libxenomai_native)
PKG_CONFIG_USE_DEPENDENCY (test-pneumaticarm7R libxenomai_vxworks)
PKG_CONFIG_USE_DEPENDENCY (test-pneumaticarm7R ncurses)

ADD_EXECUTABLE(test_ncurses_ui test_ncurses_ui.cpp  
  ${CMAKE_SOURCE_DIR}/src/ncurses_ui.cpp 
  ${CMAKE_SOURCE_DIR}/src/controller.cpp 
  ${CMAKE_SOURCE_DIR}/src/shared_memory.cpp
  ${CMAKE_SOURCE_DIR}/src/pneumaticarm_model.cpp
  ${CMAKE_SOURCE_DIR}/src/MPCcontroller.cpp
  ${CMAKE_SOURCE_DIR}/src/ilqrsolver.cpp
  ${CMAKE_SOURCE_DIR}/src/dynamicmodel.cpp
  ${CMAKE_SOURCE_DIR}/src/costfunction.cpp
  ${CMAKE_SOURCE_DIR}/src/costfunctionpneumaticarmelbow.cpp
  ${CMAKE_SOURCE_DIR}/src/pneumaticarmelbow2ndordermodel.cpp  #pneumaticarmelbowlinear.cpp
  )    


PKG_CONFIG_USE_DEPENDENCY (test_ncurses_ui ncurses)
PKG_CONFIG_USE_DEPENDENCY (test_ncurses_ui libxenomai_rtdm)
PKG_CONFIG_USE_DEPENDENCY (test_ncurses_ui libxenomai_native)
PKG_CONFIG_USE_DEPENDENCY (test_ncurses_ui libxenomai_vxworks)
TARGET_LINK_LIBRARIES(test_ncurses_ui pthread)
