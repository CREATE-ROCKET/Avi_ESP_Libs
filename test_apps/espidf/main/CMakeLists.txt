idf_component_register(SRCS "main.cpp" INCLUDE_DIRS "." REQUIRES Avi_ESP_Libs)
target_compile_options(${COMPONENT_LIB} PRIVATE "$<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>")
