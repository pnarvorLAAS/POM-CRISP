cmake_minimum_required (VERSION 3.0)
project (crisp VERSION 0.1)

set(SOURCES 
    src/UrdfParser.cpp
    src/Crisp.cpp
)

add_library(crisp ${SOURCES})
set(LIBRARY_NAME crisp)
target_link_libraries(${LIBRARY_NAME} ${ENVIRE_LIBRARIES} ${urdfdom_LIBRARIES})
SET(DEPENDENCIES "envire_core urdfdom pom-base")
SET(HEADERS_DIR "include")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 

add_library(crisp-asn1 src/CrispASN1.cpp)
set(LIBRARY_NAME crisp-asn1)
target_link_libraries(${LIBRARY_NAME} ${ENVIRE_LIBRARIES} ${urdfdom_LIBRARIES})
add_dependencies(${LIBRARY_NAME} crisp cdff_types asn1_pom_conversions)
SET(DEPENDENCIES "crisp asn1_pom_conversions")
SET(HEADERS_DIR "include")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 

## Mark executables and/or libraries for installation
install(TARGETS crisp crisp-asn1
  ARCHIVE DESTINATION lib/static
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)


