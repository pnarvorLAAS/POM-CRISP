cmake_minimum_required (VERSION 3.0)
set(TARGET_name crisp)
project (${TARGET_name} VERSION 0.1)

set(SOURCES 
    src/UrdfParser.cpp
    src/Crisp.cpp
)

add_library(${TARGET_name} ${SOURCES})
target_link_libraries(${TARGET_name} ${ENVIRE_LIBRARIES} ${urdfdom_LIBRARIES})

add_library(${TARGET_name}-asn1 src/CrispASN1.cpp)
target_link_libraries(${TARGET_name}-asn1 ${TARGET_name} ${ENVIRE_LIBRARIES} ${urdfdom_LIBRARIES})
add_dependencies(${TARGET_name}-asn1 cdff_types)




