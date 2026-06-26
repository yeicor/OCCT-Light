cmake_minimum_required(VERSION 3.20)
project(occtl_demo LANGUAGES C)

find_package(OCCTL REQUIRED COMPONENTS core topo prim)

add_executable(occtl_demo main.c)
target_link_libraries(occtl_demo PRIVATE OCCTL::core OCCTL::topo OCCTL::prim)

# OCCT uses remainder() and other math functions from libm
if(NOT WIN32)
  target_link_libraries(occtl_demo PRIVATE m)
endif()

# The demo does not use geom or higher modules directly, but the transitive
# dependency chain from core→geom→topo→prim is handled by the component
# targets' INTERFACE_LINK_LIBRARIES.  The compiler and linker flags come
# from OCCTLConfig.cmake (compile definitions, include dirs, OCCT libs).

