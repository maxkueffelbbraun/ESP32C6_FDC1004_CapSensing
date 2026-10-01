# Select one firmware entrypoint per PlatformIO environment.
set(app_sources)

# PlatformIO uses separate build directories per env (e.g. .../.pio/build/<envname>).
# Use that path to select the firmware variant source.
if(CMAKE_BINARY_DIR MATCHES "esp32-c6-devkitc-1-visual")
	list(APPEND app_sources "main_visual.cpp")
else()
	list(APPEND app_sources "main.cpp")
endif()

idf_component_register(SRCS ${app_sources})
