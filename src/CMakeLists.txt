# Create a shared library libhello.so with a single source file
set(check_gridmap_sources check_gridmap.c)

include_directories(
  ${GLIB2_INCLUDE_DIRS}
)

#message (TEST ${GLIB2_INCLUDE_DIRS})

add_definitions(
   -std=gnu99
)

add_library(check-gridmap SHARED
    check_gridmap.c)

# make the header public
# install it to include/hello
pods_install_headers(check_gridmap.h message_buffer.h DESTINATION check_gridmap)

# make the library public
pods_install_libraries(check-gridmap)

set(REQUIRED_LIBS
    lcm
    bot2-core
    bot2-frames
    bot2-lcmgl-client
    gridmap-utils
    carmen-utils
    lcmtypes_ripl-general
    lcmtypes_obstacle-detector
    lcmtypes_rrtstar
    lcmtypes_map
    gthread-2.0
    glib-2.0
    geom-utils
    map3d_interface)

pods_use_pkg_config_packages(check-gridmap ${REQUIRED_LIBS})


set(REQUIRED_PACKAGES lcm bot2-core lcmtypes_ripl-general bot2-frames bot2-lcmgl-client)

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(check-gridmap
    CFLAGS
    LIBS -lcheck-gridmap
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
