idf_component_register(REQUIRES nvs_flash)

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/eeprom.bin
				   COMMAND ${PYTHON} ${IDF_PATH}/components/nvs_flash/nvs_partition_generator/nvs_partition_gen.py generate ${CMAKE_CURRENT_SOURCE_DIR}/eeprom.csv ${CMAKE_CURRENT_SOURCE_DIR}/eeprom.bin 0x8000
				   DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/eeprom.csv )
add_custom_target(sx1276 DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/eeprom.bin)
add_dependencies(${COMPONENT_TARGET} sx1276)

set_property(DIRECTORY "${COMPONENT_PATH}" APPEND PROPERTY
     ADDITIONAL_MAKE_CLEAN_FILES ${CMAKE_CURRENT_SOURCE_DIR}/eeprom.bin)
# Set values used in flash_sx1276_args.in and generate flash file
# for sx1276
#esptool_py_flash_project_args(sx1276 0x190000 ${CMAKE_CURRENT_SOURCE_DIR}/eeprom.bin FLASH_IN_PROJECT)
esptool_py_flash_target(sx1276-flash "${main_args}" "${sub_args}")
esptool_py_flash_target_image(sx1276-flash sx1276 "0x190000" "${CMAKE_CURRENT_SOURCE_DIR}/eeprom.bin")
esptool_py_flash_target_image(flash sx1276 "0x190000" "${CMAKE_CURRENT_SOURCE_DIR}/eeprom.bin")
