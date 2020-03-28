idf_component_register(REQUIRES nvs_flash)

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/s2lp.bin
				   COMMAND ${PYTHON} ${IDF_PATH}/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py generate ${CMAKE_CURRENT_SOURCE_DIR}/nvs.csv ${CMAKE_CURRENT_SOURCE_DIR}/s2lp.bin 0x3000
				   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/nvs.csv )
add_custom_target(s2lp DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/s2lp.bin)	
add_dependencies(${COMPONENT_TARGET} s2lp)

set_property(DIRECTORY "${COMPONENT_PATH}" APPEND PROPERTY
     ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/s2lp.bin)
# Set values used in flash_s2lp_args.in and generate flash file
# for s2lp
esptool_py_flash_project_args(s2lp 0x110000 ${CMAKE_CURRENT_SOURCE_DIR}/s2lp.bin FLASH_IN_PROJECT)

