FIND_PACKAGE(Go)

set_source_files_properties (go-mraa.i PROPERTIES CPLUSPLUS on)
set_property (SOURCE go-mraa.i PROPERTY SWIG_FLAGS "-intgosize" "32" "-package" "mraa" "-use-shlib" "-I${CMAKE_BINARY_DIR}/src")
swig_add_module (go-mraa go go-mraa.i ${mraa_LIB_SRCS})

