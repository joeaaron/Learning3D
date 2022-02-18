cmake_minimum_required(VERSION 3.5)
Set(PROJECT_NAME PCLViewer)
project(${PROJECT_NAME})

set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

find_package(Qt5 REQUIRED Widgets)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)

file( GLOB header_list *.h )
file( GLOB source_list *.cpp)
file( GLOB ui_list *.ui )
file( GLOB qrc_list *.qrc )

add_executable(${PROJECT_NAME} ${header_list} ${source_list} ${ui_list} ${qrc_list} )

#include
target_include_directories(${PROJECT_NAME}
	PUBLIC
	${PCL_INCLUDE_DIRS}
)

#link
target_link_directories( ${PROJECT_NAME} 
	PUBLIC
		${PCL_LIBRARY_DIRS}
)
target_link_libraries(${PROJECT_NAME}
	${PCL_LIBRARIES}
	${VTK_LIBRARIES}
	Qt5::Widgets
)
