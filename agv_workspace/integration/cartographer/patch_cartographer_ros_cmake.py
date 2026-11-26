#!/usr/bin/env python3
"""Patch cartographer_ros 2.0.9003 CMake for Jazzy.

Upstream compiles offline_node.cpp into the shared library, which makes its
gflags definitions collide with cartographer_node's own definitions. Move
offline_node.cpp into the offline executable and add the Jazzy urdf include
directory for that target.
"""

import sys
from pathlib import Path


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    path = Path(sys.argv[1])
    if not path.is_file():
        print(f"not a file: {path}", file=sys.stderr)
        return 2

    text = path.read_text()
    text = text.replace("  src/offline_node.cpp\n", "")
    text = text.replace(
        "add_executable(cartographer_offline_node src/offline_node_main.cpp)",
        "add_executable(cartographer_offline_node src/offline_node_main.cpp src/offline_node.cpp)\n"
        "target_compile_definitions(cartographer_offline_node PRIVATE USE_URDF_H_FILES)",
    )
    old = (
        'target_include_directories(cartographer_offline_node PRIVATE\n'
        '  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"\n'
        '  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"\n'
        '  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"\n'
        ')'
    )
    new = (
        'target_include_directories(cartographer_offline_node PRIVATE\n'
        '  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"\n'
        '  "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>"\n'
        '  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"\n'
        '  ${urdf_INCLUDE_DIRS}\n'
        ')'
    )
    if old not in text:
        print("offline include block not found; CMake may already be patched", file=sys.stderr)
        return 1
    text = text.replace(old, new)
    path.write_text(text)
    print("patched cartographer_ros CMake for Jazzy")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
