cmake_minimum_required (VERSION 3.0)
project (crisp VERSION 0.1)

set(SOURCES 
    src/UrdfParser.cpp
    src/Crisp.cpp
)

set(LIBRARY_NAME crisp)
add_library(${LIBRARY_NAME} ${SOURCES})
target_link_libraries(${LIBRARY_NAME} ${ENVIRE_LIBRARIES} ${urdfdom_LIBRARIES})
SET(DEPENDENCIES "envire_core urdfdom")
SET(HEADERS_DIR "include")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 

set(LIBRARY_NAME crisp-asn1)
add_library(${LIBRARY_NAME} src/CrispASN1.cpp)
target_link_libraries(${LIBRARY_NAME} ${ENVIRE_LIBRARIES} ${urdfdom_LIBRARIES})
add_dependencies(${LIBRARY_NAME} crisp cdff_types)
SET(DEPENDENCIES "envire_core urdfdom cdff_types")
SET(HEADERS_DIR "include")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 



