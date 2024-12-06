include("${CMAKE_CURRENT_LIST_DIR}/rule.cmake")
include("${CMAKE_CURRENT_LIST_DIR}/file.cmake")
set(SCE_VCU_FreeRTOS_default_library_list )
# Handle files with suffix s 
if(SCE_VCU_FreeRTOS_default_FILE_GROUP_assemble)
    add_library(SCE_VCU_FreeRTOS_default_assemble OBJECT ${SCE_VCU_FreeRTOS_default_FILE_GROUP_assemble})
    SCE_VCU_FreeRTOS_default_assemble_rule(SCE_VCU_FreeRTOS_default_assemble)
    list(APPEND SCE_VCU_FreeRTOS_default_library_list "$<TARGET_OBJECTS:SCE_VCU_FreeRTOS_default_assemble>")
endif()

# Handle files with suffix S 
if(SCE_VCU_FreeRTOS_default_FILE_GROUP_assembleWithPreprocess)
    add_library(SCE_VCU_FreeRTOS_default_assembleWithPreprocess OBJECT ${SCE_VCU_FreeRTOS_default_FILE_GROUP_assembleWithPreprocess})
    SCE_VCU_FreeRTOS_default_assembleWithPreprocess_rule(SCE_VCU_FreeRTOS_default_assembleWithPreprocess)
    list(APPEND SCE_VCU_FreeRTOS_default_library_list "$<TARGET_OBJECTS:SCE_VCU_FreeRTOS_default_assembleWithPreprocess>")
endif()

# Handle files with suffix [cC] 
if(SCE_VCU_FreeRTOS_default_FILE_GROUP_compile)
    add_library(SCE_VCU_FreeRTOS_default_compile OBJECT ${SCE_VCU_FreeRTOS_default_FILE_GROUP_compile})
    SCE_VCU_FreeRTOS_default_compile_rule(SCE_VCU_FreeRTOS_default_compile)
    list(APPEND SCE_VCU_FreeRTOS_default_library_list "$<TARGET_OBJECTS:SCE_VCU_FreeRTOS_default_compile>")
endif()

# Handle files with suffix cpp 
if(SCE_VCU_FreeRTOS_default_FILE_GROUP_compile_cpp)
    add_library(SCE_VCU_FreeRTOS_default_compile_cpp OBJECT ${SCE_VCU_FreeRTOS_default_FILE_GROUP_compile_cpp})
    SCE_VCU_FreeRTOS_default_compile_cpp_rule(SCE_VCU_FreeRTOS_default_compile_cpp)
    list(APPEND SCE_VCU_FreeRTOS_default_library_list "$<TARGET_OBJECTS:SCE_VCU_FreeRTOS_default_compile_cpp>")
endif()

if (BUILD_LIBRARY)
        message(STATUS "Building LIBRARY")
        add_library(${SCE_VCU_FreeRTOS_default_image_name} ${SCE_VCU_FreeRTOS_default_library_list})
        foreach(lib ${SCE_VCU_FreeRTOS_default_FILE_GROUP_link})
        target_link_libraries(${SCE_VCU_FreeRTOS_default_image_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR} /${lib})
        endforeach()
        add_custom_command(
            TARGET ${SCE_VCU_FreeRTOS_default_image_name}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${SCE_VCU_FreeRTOS_default_output_dir}
    COMMAND ${CMAKE_COMMAND} -E copy lib${SCE_VCU_FreeRTOS_default_image_name}.a ${SCE_VCU_FreeRTOS_default_output_dir}/${SCE_VCU_FreeRTOS_default_original_image_name})
else()
    message(STATUS "Building STANDARD")
    add_executable(${SCE_VCU_FreeRTOS_default_image_name} ${SCE_VCU_FreeRTOS_default_library_list})
    foreach(lib ${SCE_VCU_FreeRTOS_default_FILE_GROUP_link})
    target_link_libraries(${SCE_VCU_FreeRTOS_default_image_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR}/${lib})
endforeach()
    SCE_VCU_FreeRTOS_default_link_rule(${SCE_VCU_FreeRTOS_default_image_name})
        add_custom_target(
        SCE_VCU_FreeRTOS_default_Bin2Hex
        ALL
        ${MP_BIN2HEX} ${SCE_VCU_FreeRTOS_default_image_name}
    )
    add_dependencies(SCE_VCU_FreeRTOS_default_Bin2Hex ${SCE_VCU_FreeRTOS_default_image_name})

add_custom_command(
    TARGET ${SCE_VCU_FreeRTOS_default_image_name}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${SCE_VCU_FreeRTOS_default_output_dir}
    COMMAND ${CMAKE_COMMAND} -E copy ${SCE_VCU_FreeRTOS_default_image_name} ${SCE_VCU_FreeRTOS_default_output_dir}/${SCE_VCU_FreeRTOS_default_original_image_name})
endif()
