cmake_minimum_required (VERSION 3.0)
project (crisp VERSION 0.1)

set(SOURCES 
    src/Crisp.cpp)
set(HEADERS
    include/UrdfParser.hpp
)

set(LIBRARY_NAME crisp)
add_library(${LIBRARY_NAME} ${SOURCES})
target_link_libraries(${LIBRARY_NAME} ${ENVIRE_LIBRARIES} ${urdfdom_LIBRARIES} pom-base)
add_dependencies(${LIBRARY_NAME} pom-base)
SET(DEPENDENCIES "envire_core urdfdom pom-base")
SET(HEADERS_DIR "include")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY)

# Install script
set(HEADERS_INSTALL_DIR include/crisp)
set(LIB_INSTALL_DIR lib)
include(${CMAKE_SOURCE_DIR}/CMakeModules/CDFF_at_LAAS-install.cmake)

set(SOURCES 
    src/CrispASN1.cpp)
set(HEADERS
    include/CrispASN1.hpp)
set(LIBRARY_NAME crisp-asn1)
add_library(${LIBRARY_NAME} src/CrispASN1.cpp)
target_link_libraries(${LIBRARY_NAME} ${ENVIRE_LIBRARIES} ${urdfdom_LIBRARIES} pom-base)
add_dependencies(${LIBRARY_NAME} crisp cdff_types asn1_pom_conversions pom-base)
SET(DEPENDENCIES "crisp asn1_pom_conversions")
SET(HEADERS_DIR "include")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 

# Install script
set(HEADERS_INSTALL_DIR include/crisp)
set(LIB_INSTALL_DIR lib)
include(${CMAKE_SOURCE_DIR}/CMakeModules/CDFF_at_LAAS-install.cmake)
