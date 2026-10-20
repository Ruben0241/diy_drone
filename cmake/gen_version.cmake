# Schreibt fw_version.h mit dem aktuellen Git-Commit (laeuft bei jedem Build)
execute_process(
    COMMAND git rev-parse --short=8 HEAD
    WORKING_DIRECTORY ${SRC_DIR}
    OUTPUT_VARIABLE HASH
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
    RESULT_VARIABLE RES)
if(NOT RES EQUAL 0 OR HASH STREQUAL "")
    set(HASH "00000000")
endif()
set(CONTENT "/* automatisch erzeugt */\n#pragma once\n#define FW_GIT_HASH 0x${HASH}u\n#define FW_GIT_HASH_STR \"${HASH}\"\n")
if(EXISTS ${OUT})
    file(READ ${OUT} OLD)
endif()
if(NOT "${OLD}" STREQUAL "${CONTENT}")
    file(WRITE ${OUT} "${CONTENT}")
endif()
