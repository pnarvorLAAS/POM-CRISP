cmake_minimum_required (VERSION 3.0)
project (crisp VERSION 0.1)

set(HEADERS_INSTALL_DIR include/crisp)
set(LIB_INSTALL_DIR lib)

set(SOURCES 
    src/UrdfParser.cpp
    src/Crisp.cpp
)

set(HEADERS
    include/UrdfParser.hpp
    include/Crisp.hpp)

set(LIBRARY_NAME crisp)
add_library(${LIBRARY_NAME} ${SOURCES})
target_link_libraries(${LIBRARY_NAME} ${ENVIRE_LIBRARIES} ${urdfdom_LIBRARIES})
SET(DEPENDENCIES "envire_core urdfdom pom-base")
SET(HEADERS_DIR "include")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 

## Generate and install a .pc file for installation (not the same as the one for compilation)
install(CODE "
    set(LIB_INSTALL_DIR ${LIB_INSTALL_DIR})
    set(HEADERS_INSTALL_DIR ${HEADERS_INSTALL_DIR})
    set(LIBRARY_NAME ${LIBRARY_NAME})
    set(PROJECT_VERSION ${PROJECT_VERSION})
    set(DEPENDENCIES ${DEPENDENCIES})
    foreach(header ${HEADERS})
        file(INSTALL ${CMAKE_CURRENT_SOURCE_DIR}/\${header} DESTINATION ${CMAKE_INSTALL_PREFIX}/${HEADERS_INSTALL_DIR})
    endforeach()
    configure_file(${CMAKE_SOURCE_DIR}/template-install.pc.in ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY)")
## Mark executables and/or libraries for installation
install(TARGETS ${LIBRARY_NAME}
  ARCHIVE DESTINATION ${LIB_INSTALL_DIR}/static
  LIBRARY DESTINATION ${LIB_INSTALL_DIR}
)

set(SOURCES 
    src/CrispASN1.cpp
)
set(HEADERS
    include/CrispASN1.hpp)
set(LIBRARY_NAME crisp-asn1)
add_library(${LIBRARY_NAME} ${SOURCES})
target_link_libraries(${LIBRARY_NAME} ${ENVIRE_LIBRARIES} ${urdfdom_LIBRARIES})
add_dependencies(${LIBRARY_NAME} crisp cdff_types asn1_pom_conversions)
SET(DEPENDENCIES "crisp asn1_pom_conversions")
SET(HEADERS_DIR "include")
configure_file(${CMAKE_SOURCE_DIR}/template.pc.in ${CMAKE_BINARY_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY) 

## Generate and install a .pc file for installation (not the same as the one for compilation)
install(CODE "
    set(LIB_INSTALL_DIR ${LIB_INSTALL_DIR})
    set(HEADERS_INSTALL_DIR ${HEADERS_INSTALL_DIR})
    set(LIBRARY_NAME ${LIBRARY_NAME})
    set(PROJECT_VERSION ${PROJECT_VERSION})
    set(DEPENDENCIES ${DEPENDENCIES})
    foreach(header ${HEADERS})
        file(INSTALL ${CMAKE_CURRENT_SOURCE_DIR}/\${header} DESTINATION ${CMAKE_INSTALL_PREFIX}/${HEADERS_INSTALL_DIR})
    endforeach()
    configure_file(${CMAKE_SOURCE_DIR}/template-install.pc.in ${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}/pkgconfig/${LIBRARY_NAME}.pc @ONLY)")
## Mark executables and/or libraries for installation
install(TARGETS ${LIBRARY_NAME}
  ARCHIVE DESTINATION ${LIB_INSTALL_DIR}/static
  LIBRARY DESTINATION ${LIB_INSTALL_DIR}
)


