#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "OpenArmCANOpenArmXCom::openarmx_can" for configuration ""
set_property(TARGET OpenArmCANOpenArmXCom::openarmx_can APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(OpenArmCANOpenArmXCom::openarmx_can PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libopenarmx_can.a"
  )

list(APPEND _IMPORT_CHECK_TARGETS OpenArmCANOpenArmXCom::openarmx_can )
list(APPEND _IMPORT_CHECK_FILES_FOR_OpenArmCANOpenArmXCom::openarmx_can "${_IMPORT_PREFIX}/lib/libopenarmx_can.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
