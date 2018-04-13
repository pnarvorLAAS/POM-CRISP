cmake_minimum_required (VERSION 3.5)
set(TARGET_name crisp-simple-urdf-load)
project (${TARGET_name})

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} -std=c++11) 

include_directories(../../src)

set(SOURCES ../../src/PositionManagerBase.cpp
            ../../src/UrdfParser.cpp
            ../../src/Crisp.cpp
            src/main.cpp)

find_package(PkgConfig REQUIRED)

#find_package(Threads REQUIRED)
#set(LIBS ${LIBS} Threads::Threads)

set(PACKAGE_NAMES asn1-infuse-types
                  urdfdom
                  eigen3
                  tinyxml
                  pthread
                  envire_core
                  )

foreach(package_name  ${PACKAGE_NAMES})
    pkg_check_modules(${package_name} REQUIRED ${package_name})
    include_directories(${TARGET_name} ${${package_name}_INCLUDE_DIRS})
    set(LIBS ${LIBS} ${${package_name}_LIBRARIES})
    link_directories(${${package_name}_LIBRARY_DIRS})
endforeach(package_name)

include_directories($ENV{WORK_DIR}/CDFF_at_LAAS/Common/Conversions/include/conversions/)

add_executable(${TARGET_name} ${SOURCES})
target_link_libraries(${TARGET_name} ${LIBS})





