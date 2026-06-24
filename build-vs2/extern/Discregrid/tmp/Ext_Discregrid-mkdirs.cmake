# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "D:/master/vr/final2/PositionBasedDynamics/build-vs2/extern/Discregrid/src/Ext_Discregrid")
  file(MAKE_DIRECTORY "D:/master/vr/final2/PositionBasedDynamics/build-vs2/extern/Discregrid/src/Ext_Discregrid")
endif()
file(MAKE_DIRECTORY
  "D:/master/vr/final2/PositionBasedDynamics/build-vs2/extern/Discregrid/src/Ext_Discregrid-build"
  "D:/master/vr/final2/PositionBasedDynamics/build-vs2/Discregrid"
  "D:/master/vr/final2/PositionBasedDynamics/build-vs2/extern/Discregrid/tmp"
  "D:/master/vr/final2/PositionBasedDynamics/build-vs2/extern/Discregrid/src/Ext_Discregrid-stamp"
  "D:/master/vr/final2/PositionBasedDynamics/build-vs2/extern/Discregrid/src"
  "D:/master/vr/final2/PositionBasedDynamics/build-vs2/extern/Discregrid/src/Ext_Discregrid-stamp"
)

set(configSubDirs Debug;Release;MinSizeRel;RelWithDebInfo)
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/master/vr/final2/PositionBasedDynamics/build-vs2/extern/Discregrid/src/Ext_Discregrid-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/master/vr/final2/PositionBasedDynamics/build-vs2/extern/Discregrid/src/Ext_Discregrid-stamp${cfgdir}") # cfgdir has leading slash
endif()
