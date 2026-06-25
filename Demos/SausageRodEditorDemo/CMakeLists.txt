set(SIMULATION_LINK_LIBRARIES imgui glfw PositionBasedDynamics Simulation Utils)
set(SIMULATION_DEPENDENCIES imgui glfw PositionBasedDynamics Simulation Utils CopyPBDShaders CopyPBDModels CopyImguiFonts)

if(WIN32)
  set(SIMULATION_LINK_LIBRARIES opengl32.lib glu32.lib ${SIMULATION_LINK_LIBRARIES})
else()
  set(OpenGL_GL_PREFERENCE GLVND)
  find_package(OpenGL REQUIRED)
  include_directories(${OPENGL_INCLUDE_DIRS})
  set(SIMULATION_LINK_LIBRARIES ${SIMULATION_LINK_LIBRARIES} ${OPENGL_LIBRARIES})
endif()

include_directories(${Discregrid_INCLUDE_DIR})
if (TARGET Ext_Discregrid)
	set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_Discregrid)
endif()
set(SIMULATION_LINK_LIBRARIES ${SIMULATION_LINK_LIBRARIES} ${Discregrid_LIBRARIES})

include_directories(${GenericParameters_INCLUDE_DIR})
if(TARGET Ext_GenericParameters)
	set(SIMULATION_DEPENDENCIES ${SIMULATION_DEPENDENCIES} Ext_GenericParameters)
endif()

add_executable(SausageRodEditorDemo
	  main.cpp

	  ../Common/LogWindow.cpp
	  ../Common/LogWindow.h
	  ../Common/Simulator_GUI_imgui.cpp
	  ../Common/Simulator_GUI_imgui.h
	  ../Common/imguiParameters.cpp
	  ../Common/imguiParameters.h
	  ../Common/DemoBase.cpp
	  ../Common/DemoBase.h

	  ${VIS_FILES}
	  ${PROJECT_PATH}/Common/Common.h

	  CMakeLists.txt
)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

set_target_properties(SausageRodEditorDemo PROPERTIES FOLDER "Demos")
set_target_properties(SausageRodEditorDemo PROPERTIES DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX})
set_target_properties(SausageRodEditorDemo PROPERTIES RELWITHDEBINFO_POSTFIX ${CMAKE_RELWITHDEBINFO_POSTFIX})
set_target_properties(SausageRodEditorDemo PROPERTIES MINSIZEREL_POSTFIX ${CMAKE_MINSIZEREL_POSTFIX})
add_dependencies(SausageRodEditorDemo ${SIMULATION_DEPENDENCIES})
target_link_libraries(SausageRodEditorDemo ${SIMULATION_LINK_LIBRARIES})
VIS_SOURCE_GROUPS()
