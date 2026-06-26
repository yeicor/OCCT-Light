vcpkg_minimum_required(VERSION 2022-12-15)

# === SOURCE FETCH ===
#
# Before pushing to a remote, this port uses the local checkout at a known
# path relative to the overlay location.  To use with a remote git repo
# instead (e.g. after pushing to origin), replace the block below with:
#
#   vcpkg_from_git(
#     OUT_SOURCE_PATH SOURCE_PATH
#     REPO https://github.com/Open-Cascade-SAS/OCCT-Light.git
#     REF 72b6041c7a45f4217ad50981ca3254fc183bca72
#     HEAD_REF master
#   )
#
# Update GIT_REF to match the commit you want to pin.

set(SOURCE_PATH "${CMAKE_CURRENT_LIST_DIR}/../../..")
get_filename_component(SOURCE_PATH "${SOURCE_PATH}" ABSOLUTE)

# === CONFIGURE ===
vcpkg_cmake_configure(
  SOURCE_PATH "${SOURCE_PATH}"
  OPTIONS
    -DOCCTL_BUILD_TESTING=OFF
    -DOCCTL_BUILD_VIZ=OFF
    -DOCCTL_BUILD_BINDINGS_CSHARP=OFF
    -DOCCTL_BUILD_BINDINGS_PYTHON=OFF
    -DOCCTL_BUILD_BINDINGS_WASM=OFF
    -DOCCTL_BUILD_GEOM=ON
    -DOCCTL_BUILD_TOPO=ON
    -DOCCTL_BUILD_PRIM=ON
)

# === BUILD & INSTALL ===
vcpkg_cmake_install()

# === FIXUP CONFIG ===
vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake/OCCTL)

# === COPY LICENCE ===
vcpkg_install_copyright(FILE_LIST "${SOURCE_PATH}/LICENSE_AGPL_30.txt")

# === CLEANUP ===
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include")
file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/share")
