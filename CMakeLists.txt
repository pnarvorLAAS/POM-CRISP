cmake_minimum_required(VERSION 3.0)

####
# Set variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
project(infuse_pom_crisp VERSION 0.1.2)

set(CMAKE_CXX_STANDARD 11)
set(PKG_DESCRIPTION "CRISP: Common Robot Internal State Publisher")

###
# Imports infuse_pom_base dependencies

find_package(infuse_pom_base REQUIRED )

#get_property(pom_base_link TARGET infuse_pom_base PROPERTY INTERFACE_LINK_LIBRARIES)
#get_property(pom_base_include TARGET infuse_pom_base PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
#message(STATUS "pom_base Link: ${pom_base_link}")
#message(STATUS "pom_base Include: ${pom_base_include}")


###
# Define options
option (WITH_ASN1 "Compile the ASN.1 Wrapper of pom-crisp" TRUE)

####
# If the ASN.1 wrapper is enabled
if (WITH_ASN1)
    find_package(infuse_asn1_conversions REQUIRED)
    #include_directories(${infuse_asn1_conversions_INCLUDE_DIR})
#    get_property(conversions_link TARGET infuse_asn1_conversions::pom_conversions PROPERTY INTERFACE_LINK_LIBRARIES)
#    get_property(conversions_include TARGET infuse_asn1_conversions::pom_conversions PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
#
#    message(STATUS "conversions Link: ${conversions_link}")
#    message(STATUS "conversions Include: ${conversions_include}")


    set(SOURCES 
        src/KinematicChain.cpp
        src/Crisp.cpp
        src/CrispASN1.cpp
        )

    set(HEADERS
        include/infuse_pom_crisp/KinematicChain.hpp
        include/infuse_pom_crisp/Crisp.hpp
        include/infuse_pom_crisp/CrispASN1.hpp
        )
    
    add_library(${PROJECT_NAME} SHARED ${SOURCES})
    
    target_link_libraries (${PROJECT_NAME} infuse_pom_base infuse_asn1_conversions::pom_conversions)
    target_include_directories(${PROJECT_NAME} PRIVATE include/)
    set_target_properties (${PROJECT_NAME}
        PROPERTIES
            VERSION ${PROJECT_VERSION}
            OUTPUT_NAME ${PROJECT_NAME}
            PUBLIC_HEADER "${HEADERS}"
    )

    set(REQUIRED_COMPONENTS "infuse_pom_base, infuse_asn1_conversions_pom")

###
# Else

else()
    message(STATUS "\n\n[ATTENTION] You're currently compiling CRISP without ASN.1 support. You wil need to code your own network interface in order to communicate with CRISP \n\n")

    set(SOURCES 
        src/KinematicChain.cpp
        src/Crisp.cpp
        )

    set(HEADERS
        include/infuse_pom_crisp/KinematicChain.hpp
        include/infuse_pom_crisp/Crisp.hpp
        )
    
    add_library(${PROJECT_NAME} SHARED ${SOURCES})
    target_include_directories(${PROJECT_NAME} PRIVATE include/)
    target_link_libraries (${PROJECT_NAME} infuse_envire infuse_pom_base)
    set_target_properties (${PROJECT_NAME}
        PROPERTIES
            VERSION ${PROJECT_VERSION}
            OUTPUT_NAME ${PROJECT_NAME}
            PUBLIC_HEADER "${HEADERS}"
    )
    
    set(REQUIRED_COMPONENTS "infuse_envire, infuse_pom_base")
    

endif(WITH_ASN1)


#####
# Generation of configuration files


# Introduce variables:
# * CMAKE_INSTALL_LIBDIR
# * CMAKE_INSTALL_BINDIR
# * CMAKE_INSTALL_INCLUDEDIR
include(GNUInstallDirs)


# Layout. This works for all platforms:
#   * <prefix>/lib*/cmake/<PROJECT-NAME>
#   * <prefix>/lib*/
#   * <prefix>/include/
set(config_install_dir "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

# This is exclusive for pkg-config
set(pkgconfig_install_dir "${CMAKE_INSTALL_LIBDIR}/pkgconfig")


set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")


# Configuration
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(pkgconfig_file "${generated_dir}/${PROJECT_NAME}.pc")
set(TARGETS_EXPORT_NAME "${PROJECT_NAME}Targets")
#
#
# Include module with fuctions:
#   * 'write_basic_package_version_file'
#   * 'configure_package_config_file'
include(CMakePackageConfigHelpers)
#
#
# Configure '<PROJECT-NAME>ConfigVersion.cmake'
# Uses:
#   * PROJECT_VERSION
write_basic_package_version_file(
  "${version_config}" COMPATIBILITY SameMajorVersion
)

# Configure '<PROJECT-NAME>Config.cmake'
# Uses variables:
#   * TARGETS_EXPORT_NAME
#   * PROJECT_NAME
#   * CMAKE_INSTALL_INCLUDEDIR
configure_package_config_file(
    "cmake/Config.cmake.in"
    "${project_config}"
    INSTALL_DESTINATION "${config_install_dir}"
    PATH_VARS
      CMAKE_INSTALL_INCLUDEDIR
)

# Configure '<PROJECT-NAME>.pc'
# Uses variables:
#   * PROJECT_NAME
#   * PROJECT_VERSION
#   * CMAKE_INSTALL_PREFIX
#   * CMAKE_INSTALL_LIBDIR
#   * CMAKE_INSTALL_INCLUDEDIR
configure_file(
    "cmake/pkgconfig.pc.cmakein"
    "${pkgconfig_file}" @ONLY
)


####
# Installation


# Targets:
#   * <prefix>/lib/lib${PROJECT_NAME}.so
#   * header location after install: <prefix>/include/${PROJECT_NAME}/*.h
#   * headers can be included by C++ code `#include <${PROJECT_NAME}/*.h>`
install(
    TARGETS ${PROJECT_NAME}
    EXPORT "${TARGETS_EXPORT_NAME}"
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}"
)


# Config
#   * <prefix>/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}Config.cmake
#   * <prefix>/lib/cmake/${PROJECT_NAME}/${PROJECT_NAME}ConfigVersion.cmake
install(
    FILES "${project_config}" "${version_config}"
    DESTINATION "${config_install_dir}"
)


# Config
#   * <prefix>/lib/pkgconfig/${PROJECT_NAME}.pc
install(
    FILES "${pkgconfig_file}"
    DESTINATION "${pkgconfig_install_dir}")


# Config
#   * <prefix>/lib/cmake/infuse_cdff_types/infuse_cdff_typesTargets.cmake
install(
    EXPORT "${TARGETS_EXPORT_NAME}"
    DESTINATION "${config_install_dir}"
)
