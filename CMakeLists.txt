# Copyright 2014, CNRS
#
# Author: Ganesh Kumar
#
# This file is part of vsa1.
# pam is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# vsa1 is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# pam. If not, see <http://www.gnu.org/licenses/>.

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/eigen.cmake)
#INCLUDE(cmake/findxenomai.cmake)

SET(PROJECT_NAME pneumaticarm7R)
SET(PROJECT_DESCRIPTION
  "jereme software compilation.")
SET(PROJECT_URL "")

SET(CUSTOM_HEADER_DIR "${PROJECT_NAME}")

SET(${PROJECT_NAME}_HEADERS

	include/actuator.hh
	include/sensor.hh
	include/position_sensor.hh
	include/ioboards.hh
	include/CIODAC16.hh
	include/CIODAS64.hh
	include/clientudp3.hh
	include/controller_axis.hh
	include/controller_tool.hh
	include/fichier.h
	include/I_teleop.h
	include/joystick.h
	include/modele.h
	include/port.hh
	#include/main_controleur.hh
  #include/test_config.hh

)
SET(CXX_DISABLE_WERROR TRUE)

ADD_REQUIRED_DEPENDENCY ("libxenomai_native")
ADD_REQUIRED_DEPENDENCY ("libxenomai_rtdm")
ADD_REQUIRED_DEPENDENCY ("libxenomai_vxworks")
ADD_REQUIRED_DEPENDENCY ("ncurses")

SETUP_PROJECT()

#SEARCH_FOR_BOOST()
SEARCH_FOR_EIGEN()

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(application)
SETUP_PROJECT_FINALIZE()
