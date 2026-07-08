file(GLOB_RECURSE APP_SOURCES CONFIGURE_DEPENDS "*.c" "*.cpp" "*.s")
add_library(App_Module STATIC ${APP_SOURCES})

file(GLOB_RECURSE APP_H_FILES CONFIGURE_DEPENDS "*.h" "*.hpp")
set(APP_INCLUDE_DIRS "")
foreach(FILE_PATH ${APP_H_FILES})
    get_filename_component(DIR_PATH ${FILE_PATH} DIRECTORY)
    list(APPEND APP_INCLUDE_DIRS ${DIR_PATH})
endforeach()
if(APP_INCLUDE_DIRS)
    list(REMOVE_DUPLICATES APP_INCLUDE_DIRS)
endif()

target_include_directories(App_Module PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${APP_INCLUDE_DIRS}
)

target_link_libraries(App_Module PRIVATE stm32cubemx)

get_target_property(FW_INCLUDES ${CMAKE_PROJECT_NAME} INCLUDE_DIRECTORIES)
if(FW_INCLUDES)
    target_include_directories(App_Module PRIVATE ${FW_INCLUDES})
endif()

target_link_libraries(${CMAKE_PROJECT_NAME} App_Module)