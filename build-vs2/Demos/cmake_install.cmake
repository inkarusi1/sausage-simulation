# Install script for directory: D:/master/vr/final2/PositionBasedDynamics/Demos

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files/PositionBasedDynamics")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/Demos" TYPE DIRECTORY FILES "D:/master/vr/final2/PositionBasedDynamics/Demos/./Common" FILES_MATCHING REGEX "/[^/]*\\.h$")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/BarDemo/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/ClothDemo/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/CosseratRodsDemo/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/CouplingDemos/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/DistanceFieldDemos/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/FluidDemo/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/GenericConstraintsDemos/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/PositionBasedElasticRodsDemo/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/RigidBodyDemos/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/SausageRodEditorDemo/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/SausageRodCourseDemo/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/SausagePhysicsCourseDemo/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/SceneLoaderDemo/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/SausageCourseDemo/cmake_install.cmake")
  include("D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/StiffRodsDemos/cmake_install.cmake")

endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "D:/master/vr/final2/PositionBasedDynamics/build-vs2/Demos/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
