set(COMPONENT_NAME "Data")

set (SUBMODULES 
    ${COMPONENT_NAME} 
    PARENT_SCOPE
)

message("Add sub module: ${COMPONENT_NAME}")

file(GLOB SUB_SOURCE *.cpp )
file(GLOB SUB_HEADERS *.h)

message("Add sub sources of Module: ${COMPONENT_NAME}:")
message("${SUB_SOURCE}")

message("Add sub headers of Module: ${COMPONENT_NAME}:")
message("${SUB_HEADERS}")

set(SOURCE
   ${SOURCE}
   ${SUB_SOURCE}
   PARENT_SCOPE
)

set(HEADERS
   ${HEADERS}
   ${SUB_HEADERS}
   PARENT_SCOPE
)
#Add Local Dependencies
set(LOCAL_DEPENDENCIES)
set(LOCAL_LINKLIBS)

#Create Libname list
list(APPEND LIBNAMES)

#add libraries if needed
SET(LIBNAME qrcode_referencer_data)
list(APPEND LIBNAMES ${LIBNAME})
add_library(${LIBNAME} QRCodeData.cpp)
add_dependencies(${LIBNAME} ${GLOB_DEPENDENCIES} ${LOCAL_DEPENDENCIES})
target_link_libraries(${LIBNAME} ${GLOB_LINKLIBS} ${LOCAL_LINKLIBS}) 

set(GLOB_DEPENDENCIES
    ${GLOB_DEPENDENCIES} 
    ${LIBNAME} 
    PARENT_SCOPE
)

set(GLOB_LINKLIBS
   ${GLOB_LINKLIBS}
   ${LIBNAMES}
   PARENT_SCOPE
)


