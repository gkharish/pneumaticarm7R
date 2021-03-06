# Copyright 2014, CNRS
#
# Author: Ganesh Kumar
#
# This file is part of pam .
# pam is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# pam is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# pam. If not, see <http://www.gnu.org/licenses/>.

# prefix and suffix each element of list by ${prefix}elemnt${suffix}
macro(ADDPREFIX newlist prefix list_name)
    # create empty list - necessary?
    SET(${newlist})

     # prefix and suffix elements
    foreach(l ${${list_name}})
      list(APPEND ${newlist} ${prefix}${l} )
    endforeach()

endmacro(ADDPREFIX)

ADDPREFIX(${PROJECT_NAME}_ABSOLUTE_HEADERS  "${CMAKE_SOURCE_DIR}/"  ${PROJECT_NAME}_HEADERS)

SET(LIBRARY_NAME ${PROJECT_NAME})

#INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  actuator.cpp
  sensor.cpp
  position_sensor.cpp
  CIODAC16.cpp
  CIODAS64.cpp
  clientudp3.cpp
  controller.cpp
  controller_axis.cpp
  controller_tool.cpp
  fichier.cpp
  I_teleop.cpp
  joystick.cpp
  test_config.cpp
  modele.cpp
  muscle.cpp
  shared_memory.cpp
  pneumatic_7arm_rt_thread.cpp
  pneumaticarm_model.cpp
${${PROJECT_NAME}_ABSOLUTE_HEADERS}
  )

SET_TARGET_PROPERTIES(${LIBRARY_NAME}
  PROPERTIES
  SOVERSION ${PROJECT_VERSION}
  INSTALL_RPATH ${CMAKE_INSTALL_PREFIX}/lib)

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} "libxenomai_vxworks")

INSTALL(TARGETS ${LIBRARY_NAME}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.0.0")
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIRS})
SET(LINEAR_ALGEBRA_LIB_FOUND 1)
SET(_EIGEN_MATRIX_FOUND_ 1)
