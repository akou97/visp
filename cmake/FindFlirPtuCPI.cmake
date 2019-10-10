# See http://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Try to find Kinova Jaco Robot SDK
#
# FLIRPTUCPI_FOUND
# FLIRPTUCPI_INCLUDE_DIRS
# FLIRPTUCPI_LIBRARIES
#
# Authors:
# AKOURIM Youness (ENS Rennes)
#
#############################################################################

set(FLIRPTUCPI_INC_SEARCH_PATH /usr/include)
list(APPEND FLIRPTUCPI_INC_SEARCH_PATH /usr/include/cerial)
list(APPEND FLIRPTUCPI_INC_SEARCH_PATH "$ENV{FLIRPTUCPI_HOME}")
list(APPEND FLIRPTUCPI_INC_SEARCH_PATH "$ENV{FLIRPTUCPI_HOME}/cerial")
list(APPEND FLIRPTUCPI_INC_SEARCH_PATH "C:/Users/avimucam/Documents/130108_1122/sdk-2.0.2/")
list(APPEND FLIRPTUCPI_INC_SEARCH_PATH "C:/Users/avimucam/Documents/130108_1122/sdk-2.0.2/cerial")
set(FLIRPTUCPI_LIB_SEARCH_PATH /usr/lib)

if(MSVC)
  if(CMAKE_CL_64)
    list(APPEND FLIRPTUCPI_LIB_SEARCH_PATH "C:/Users/avimucam/Documents/130108_1122/sdk-2.0.2/build/visual_studio_2008/x64/Release")
    list(APPEND FLIRPTUCPI_LIB_SEARCH_PATH "$ENV{FLIRPTUCPI_HOME}/build/visual_studio_2008/x64/Release")
  else()
    list(APPEND FLIRPTUCPI_LIB_SEARCH_PATH "C:/Users/avimucam/Documents/130108_1122/sdk-2.0.2/build/visual_studio_2008/Release")
    list(APPEND FLIRPTUCPI_LIB_SEARCH_PATH "$ENV{FLIRPTUCPI_HOME}/build/visual_studio_2008/Release")
  endif()
endif()

find_path(FLIRPTUCPI_INCLUDE_DIRS cpi.h
  PATHS  
    ${FLIRPTUCPI_INC_SEARCH_PATH}
)

find_path(FLIRPTUCERIAL_INCLUDE_DIRS cerial.h
  PATHS  
    ${FLIRPTUCPI_INC_SEARCH_PATH}
)

find_library(FLIRPTUCPI_LIBRARIES
  NAMES CPI2.lib
  PATHS 
    ${FLIRPTUCPI_LIB_SEARCH_PATH}
)

if(FLIRPTUCPI_LIBRARIES AND FLIRPTUCPI_INCLUDE_DIRS AND FLIRPTUCERIAL_INCLUDE_DIRS)
  list(APPEND FLIRPTUCPI_INCLUDE_DIRS ${FLIRPTUCERIAL_INCLUDE_DIRS})
  set(FLIRPTUCPI_FOUND TRUE)
else()
  set(FLIRPTUCPI_FOUND FALSE)
endif()

mark_as_advanced(
  FLIRPTUCPI_INCLUDE_DIRS
  FLIRPTUCERIAL_INCLUDE_DIRS
  FLIRPTUCPI_LIBRARIES
  FLIRPTUCPI_INC_SEARCH_PATH
  FLIRPTUCPI_LIB_SEARCH_PATH
)