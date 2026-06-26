# Using OCCT-Light as a vcpkg Dependency

This tutorial walks through adding OCCT-Light as a **static** dependency in your own C/C++ project using vcpkg manifest mode.

> **Git origin:** https://github.com/Open-Cascade-SAS/OCCT-Light (or your fork)  
> **OCCT version required:** 8.0.0-p1 (includes `BRepGraph` APIs not present in 8.0.0)  
> **License:** AGPL-3.0-or-later (see `LICENSE_AGPL_30.txt`)

---

## Prerequisites

| Tool      | Minimum version |
|-----------|-----------------|
| CMake     | 3.23            |
| C/C++     | C11 / C++17     |
| vcpkg     | 2026-05-27+     |
| Ninja     | (recommended)   |

Install vcpkg if you don't have it:

```bash
git clone https://github.com/microsoft/vcpkg.git /path/to/vcpkg
/path/to/vcpkg/bootstrap-vcpkg.sh
```

---

## 1. Create the OCCT 8.0.0-p1 Overlay Port

OCCT-Light requires OCCT 8.0.0-p1 (the `V8_0_0_p1` tag provides `BRepGraph_CacheDerivedState`, `BRepGraph_ItemId`, `BRepGraph_LayerHistory`, etc.). The official vcpkg registry does not ship this version, so you need an **overlay port**.

Create the overlay port directory:

```bash
mkdir -p /path/to/overlays/opencascade8p1
```

### `vcpkg.json`

```json
{
  "name": "opencascade",
  "version": "8.0.0.1",
  "description": "Open CASCADE Technology (OCCT) 8.0.0-p1 release",
  "homepage": "https://github.com/Open-Cascade-SAS/OCCT",
  "license": "LGPL-2.1-only",
  "supports": "!xbox",
  "dependencies": [
    { "name": "opengl", "platform": "!(android | ios | uwp | wasm32)" },
    { "name": "vcpkg-cmake", "host": true },
    { "name": "vcpkg-cmake-config", "host": true }
  ],
  "default-features": [ { "name": "freetype", "platform": "!uwp" } ],
  "features": {
    "freeimage": { "description": "FreeImage support", "dependencies": ["freeimage"] },
    "freetype": {
      "description": "FreeType support",
      "supports": "!uwp",
      "dependencies": [
        { "name": "fontconfig", "platform": "!android & !emscripten & !ios & !osx & !windows" },
        { "name": "freetype", "default-features": false }
      ]
    },
    "rapidjson": { "description": "RapidJSON support", "dependencies": ["rapidjson"] },
    "tbb": { "description": "TBB support", "dependencies": ["tbb"] },
    "vtk": { "description": "VTK support", "dependencies": [{ "name": "vtk", "default-features": false, "features": ["opengl"] }] }
  }
}
```

> **Version choice:** `8.0.0.1` sorts higher than `8.0.0` so it satisfies `>= 8.0.0` constraints. The `.1` suffix also prevents vcpkg from interfering with the download URL (vcpkg converts dots to underscores and would append a `1` suffix, breaking the GitHub tag name `V8_0_0_p1`).

### `portfile.cmake`

```cmake
set(VERSION_STR "V8_0_0_p1")
vcpkg_from_github(
    OUT_SOURCE_PATH SOURCE_PATH
    REPO Open-Cascade-SAS/OCCT
    REF "${VERSION_STR}"
    SHA512 f150f73a5b0cfd202838465d4fffabfc1177b1edbf175a1fa375bcec575896a35b22422bee711d5ae948c4fc242a0a89ed68f1a45a693c2b54b9b8326eabf669
    HEAD_REF master
    PATCHES
        0001-cmake-keep-build-use-vcpkg-explicit.patch
        0002-cmake-load-exported-package-dependencies.patch
        0003-image-remove-freeimage-msvc-autolink.patch
        0004-cmake-add-additional-path-extraction-for-OpenCASCADE.patch
        0005-drop-bin-letter.patch
)

if (VCPKG_LIBRARY_LINKAGE STREQUAL "dynamic")
    set(BUILD_TYPE "Shared")
else()
    set(BUILD_TYPE "Static")
endif()

vcpkg_check_features(OUT_FEATURE_OPTIONS FEATURE_OPTIONS
    FEATURES
        freeimage   USE_FREEIMAGE
        freetype    USE_FREETYPE
        rapidjson   USE_RAPIDJSON
        tbb         USE_TBB
        vtk         USE_VTK
)

vcpkg_cmake_configure(
    SOURCE_PATH "${SOURCE_PATH}"
    OPTIONS
        ${FEATURE_OPTIONS}
        -DBUILD_LIBRARY_TYPE=${BUILD_TYPE}
        -DBUILD_MODULE_Draw=OFF
        -DBUILD_DOC_Overview=OFF
        -DINSTALL_DIR_LAYOUT=Unix
        -DINSTALL_DIR_DOC=share/trash
        -DINSTALL_DIR_SCRIPT=share/trash
        -DINSTALL_TEST_CASES=OFF
        -DUSE_TK=OFF
)

vcpkg_cmake_install()
vcpkg_copy_pdbs()
vcpkg_cmake_config_fixup(CONFIG_PATH lib/cmake/opencascade)

file(GLOB extra_headers
    LIST_DIRECTORIES false
    RELATIVE "${CURRENT_PACKAGES_DIR}/include/opencascade"
    "${CURRENT_PACKAGES_DIR}/include/opencascade/*.h")
list(JOIN extra_headers "|" extra_headers)
file(GLOB files "${CURRENT_PACKAGES_DIR}/include/opencascade/*.[hgl]xx")
foreach(file_name IN LISTS files)
    vcpkg_replace_string("${file_name}" "(# *include) <([a-zA-Z0-9_]*[.][hgl]xx|${extra_headers})>" [[\1 "\2"]] REGEX IGNORE_UNCHANGED)
endforeach()

if(VCPKG_LIBRARY_LINKAGE STREQUAL "static")
    vcpkg_replace_string("${CURRENT_PACKAGES_DIR}/include/opencascade/Standard_Macro.hxx"
                          "defined(OCCT_STATIC_BUILD)" "(1)")
endif()

file(REMOVE_RECURSE "${CURRENT_PACKAGES_DIR}/debug/include"
                    "${CURRENT_PACKAGES_DIR}/debug/share"
                    "${CURRENT_PACKAGES_DIR}/share/opencascade/samples/qt"
                    "${CURRENT_PACKAGES_DIR}/share/trash")

vcpkg_install_copyright(FILE_LIST
    "${SOURCE_PATH}/LICENSE_LGPL_21.txt"
    "${SOURCE_PATH}/OCCT_LGPL_EXCEPTION.txt")
```

### Patch files

Copy the five patch files from the [upstream vcpkg opencascade port](https://github.com/microsoft/vcpkg/tree/master/ports/opencascade):

| Patch | Purpose |
|-------|---------|
| `0001-cmake-keep-build-use-vcpkg-explicit.patch` | Disables auto-detection of vcpkg toolchain |
| `0002-cmake-load-exported-package-dependencies.patch` | Loads freetype/vtk via `find_package` |
| `0003-image-remove-freeimage-msvc-autolink.patch` | Removes MSVC `#pragma comment(lib)` |
| `0004-cmake-add-additional-path-extraction-for-OpenCASCADE.patch` | Fixes config file path detection |
| `0005-drop-bin-letter.patch` | Removes debug/relwithdebinfo suffix letter from output |

Download them:

```bash
VCPKG_COMMIT="59acc80b23"
BASE="https://raw.githubusercontent.com/microsoft/vcpkg/$VCPKG_COMMIT/ports/opencascade"
for p in 0001-cmake-keep-build-use-vcpkg-explicit \
         0002-cmake-load-exported-package-dependencies \
         0003-image-remove-freeimage-msvc-autolink \
         0004-cmake-add-additional-path-extraction-for-OpenCASCADE \
         0005-drop-bin-letter; do
  curl -sL "$BASE/$p.patch" -o "/path/to/overlays/opencascade8p1/$p.patch"
done
```

---

## 2. Create Your Consumer Project

### Directory layout

```
my-app/
├── CMakeLists.txt
├── vcpkg.json
├── vcpkg-configuration.json    (optional, for overlay ports)
└── src/
    └── main.c
```

### `vcpkg.json`

```json
{
  "name": "my-app",
  "version": "1.0.0",
  "dependencies": [
    {
      "name": "occtl",
      "version>=": "0.1.0"
    }
  ],
  "builtin-baseline": "6da16a9600492fbcb7713560d993ac76823290de"
}
```

### `vcpkg-configuration.json` (register the overlay)

```json
{
  "overlay-ports": [
    "/path/to/overlays/opencascade8p1",
    "/path/to/occtl-source-directory"
  ]
}
```

> This tells vcpkg to look in the overlay directories for `opencascade` and `occtl` before falling back to the builtin registry. Both the OCCT overlay port and the OCCT-Light source tree are registered here.

### `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.23)
project(my-app VERSION 1.0.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# --- Find OCCT-Light -------------------------------------------------------
find_package(OCCTL REQUIRED COMPONENTS core topo prim mesh)

# --- Executable ------------------------------------------------------------
add_executable(my-app src/main.c)

target_link_libraries(my-app PRIVATE
    OCCTL::core
    OCCTL::topo
    OCCTL::prim
    OCCTL::mesh
)
```

#### Available components

| Component   | Description |
|-------------|-------------|
| `core`      | Foundation types, errors, runtime info |
| `geom`      | Geometry (curves, surfaces, transformations) |
| `topo`      | Topology (shape graph with `BRepGraph`) |
| `prim`      | Primitive solid/feature/sweep creation |
| `text`      | Text (brep font) |
| `bool`      | Boolean operations |
| `mesh`      | Mesh generation |
| `heal`      | Shape healing |
| `io_brep`   | BREP file I/O |
| `io_step`   | STEP file I/O |
| `io_iges`   | IGES file I/O |
| `io_stl`    | STL file I/O |
| `de`        | Data Exchange (multi-format) |
| `io_obj`    | OBJ file I/O |
| `io_gltf`   | glTF file I/O |
| `io_vrml`   | VRML file I/O |
| `io_ply`    | PLY file I/O |

Request only the ones you need; OCCT-Light will pull in only the required OCCT toolkits.

---

## 3. Build Your Project

```bash
cmake -S /path/to/my-app \
      -B /path/to/my-app/build \
      -G Ninja \
      -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      -DCMAKE_TOOLCHAIN_FILE=/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake

cmake --build /path/to/my-app/build
```

What happens during CMake configure:

1. **vcpkg installs dependencies** — `occtl` (from the overlay) and transitively `opencascade` (from the overlay), `rapidjson`, `freetype`, etc.
2. **`find_package(OCCTL)` runs** — loads `OCCTLConfig.cmake` from the vcpkg-installed occtl, which:
   - Finds OCCT via `FindOCCT.cmake`
   - Imports OCCT toolkit targets
   - Imports `OCCTLTargets.cmake` (defines `OCCTL::core`, etc.)
3. **Your app links** — `target_link_libraries(... OCCTL::core OCCTL::topo)` adds the correct include paths, compile definitions, and the static `libocctl-*.a` plus all required OCCT static libraries.

---

## 4. Minimal Usage Example

### `src/main.c`

```c
#include <occtl/occtl.h>
#include <occtl/occtl_topo.h>
#include <occtl/occtl_prim_solid.h>
#include <stdio.h>

int main(void)
{
    // Print version info
    occtl_version_t aVer;
    occtl_version(&aVer);
    printf("OCCT-Light v%u.%u.%u (ABI v%u)\n",
           aVer.major, aVer.minor, aVer.patch, aVer.abi);

    printf("OCCT v%s\n", occtl_occt_version());

    // Create a box
    occtl_graph_t*  aGraph = NULL;
    occtl_node_id_t aBody  = OCCTL_NODE_NONE;
    occtl_status_t  aStatus = occtl_make_box(10.0, 20.0, 30.0, &aGraph, &aBody);
    if (aStatus != OCCTL_OK) {
        fprintf(stderr, "Failed to create box: %d\n", aStatus);
        return 1;
    }

    printf("Created box (body id = %u, graph has %u shapes)\n",
           aBody, occtl_graph_size(aGraph));

    occtl_graph_free(aGraph);
    return 0;
}
```

### Build and run

```bash
cmake --build /path/to/my-app/build
./build/my-app
# Output:
#   OCCT-Light v0.1.0 (ABI v1)
#   OCCT v8.0.0
#   Created box (body id = 1, graph has 11 shapes)
```

---

## 5. Static Linking Details

When building OCCT-Light itself, `OCCTL_SHARED_LIBS=OFF` (the default) produces a **single static library** (e.g., `libocctl-full.a`) that aggregates all enabled module object files. Your consumer project links this one `.a` plus all required OCCT static libraries (`libTKernel.a`, `libTKMath.a`, `libTKBRep.a`, etc.).

The `OCCTLTargets.cmake` export handles all of this automatically:

```cmake
# From OCCTLTargets.cmake (pseudo-code):
#   target_link_libraries(OCCTL::occtl
#       INTERFACE $<LINK_ONLY:TKernel> $<LINK_ONLY:TKMath> ...
#                 $<LINK_ONLY:Threads::Threads>)
#
#   target_link_libraries(OCCTL::topo
#       INTERFACE OCCTL::occtl OCCTL::core OCCTL::geom)
#
# When you do target_link_libraries(my-app OCCTL::topo):
#   → links libocctl-full.a, all OCCT static libs, pthreads
#   → adds -DOCCTL_STATIC_BUILD -DOCCTL_HAS_TOPO ...
#   → adds include paths for occtl and opencascade headers
```

---

## 6. Verifying the Installation

After building, check the pkg-config-like metadata:

```bash
# Which components were built?
cat /path/to/vcpkg_installed/x64-linux/share/occtl/OCCTLFeatures.json

# Which exact static libraries are linked?
strings build/my-app | grep libocctl
```

---

## 7. Troubleshooting

### `gp_Trsf::SetScale` with zero factor doesn't throw

OCCT 8.0.0-p1 is built with `-DNo_Exception` in the vcpkg overlay port, which disables all OCCT exception throwing. The `occtl_transform_scale` function checks for zero factor explicitly via `gp::Resolution()` — this is already handled.

### `Standard_Failure` not caught in static builds

OCCT-Light's `Guard` wrapper catches `std::exception` and `...` at the top level, so any uncaught OCCT exception is translated to `OCCTL_INTERNAL`. Module-specific code (like geometry transforms) uses explicit precondition checks rather than relying on exception handlers.

### Conflicting vcpkg versions

If you see `opencascade` being resolved to version `8.0.0` instead of `8.0.0.1`, ensure:
1. The overlay port path in `vcpkg-configuration.json` is an **absolute path**
2. The overlay port's `vcpkg.json` version `8.0.0.1` satisfies your `version>=` constraint
3. No `overrides` block in `vcpkg.json` pins `opencascade` to `8.0.0`

---

## 8. Moving to CI

For CI/CD, cache the vcpkg binary archives:

```bash
export VCPKG_BINARY_SOURCES="files,/path/to/cache,readwrite"
```

Add the overlay port checkout step:

```yaml
# GitHub Actions example
- name: Checkout OCCT-Light
  uses: actions/checkout@v4
  with:
    repository: Open-Cascade-SAS/OCCT-Light
    path: occtl

- name: Checkout vcpkg overlay
  run: |
    mkdir -p overlays/opencascade8p1
    # ... create port files as in Section 1 ...

- name: Configure
  run: |
    cmake -S src -B build \
      -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake \
      -DVCPKG_OVERLAY_PORTS="$PWD/overlays/opencascade8p1;$PWD/occtl"
```

---

## Summary

```
┌─────────────────────────────────────────────────────┐
│                   my-app                            │
│  target_link_libraries(PRIVATE                      │
│      OCCTL::core                                    │
│      OCCTL::topo                                    │
│      OCCTL::prim)                                   │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│               OCCT-Light (vcpkg overlay)            │
│  libocctl-full.a  +  occtl headers                  │
│  OCCTL::core, OCCTL::topo, ... interface targets    │
└──────────────────────┬──────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────┐
│          OCCT 8.0.0-p1 (vcpkg overlay)              │
│  libTKernel.a, libTKMath.a, libTKBRep.a, ...        │
│  BRepGraph_CacheDerivedState, ...                    │
└─────────────────────────────────────────────────────┘
```

You now have a fully static, hermetic build of OCCT-Light + OCCT 8.0.0-p1 managed entirely through vcpkg manifest mode. To update OCCT-Light, simply point the overlay to a newer checkout and rebuild.
