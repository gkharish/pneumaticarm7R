# provide path to dependency libraries

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
#INCLUDE_DIRECTORIES(${XENOMAI_RTDM_INCLUDE_DIR})

LINK_DIRECTORIES(${${PROJECT_NAME}_BINARY_DIR}/src)
LINK_DIRECTORIES(/usr/xenomai/lib)

ADD_EXECUTABLE(test-pneumaticarm7R main_controleur.cpp)
TARGET_LINK_LIBRARIES(test-pneumaticarm7R pneumaticarm7R)
PKG_CONFIG_USE_DEPENDENCY (test-pneumaticarm7R libxenomai_rtdm)
PKG_CONFIG_USE_DEPENDENCY (test-pneumaticarm7R libxenomai_native)
PKG_CONFIG_USE_DEPENDENCY (test-pneumaticarm7R libxenomai_vxworks)
PKG_CONFIG_USE_DEPENDENCY (test-pneumaticarm7R ncurses)

#ADD_EXECUTABLE(test_ncurses_ui test_ncurses_ui.cpp  ${CMAKE_SOURCE_DIR}/src/ncurses_ui.cpp)
#PKG_CONFIG_USE_DEPENDENCY (test_ncurses_ui ncurses)
#TARGET_LINK_LIBRARIES(test_ncurses_ui pthread)
