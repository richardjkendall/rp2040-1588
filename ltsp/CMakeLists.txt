# ============================================================================
# LTSP - Lightweight Statistical Time Sync Protocol
# ============================================================================

# Common LTSP library (shared between GM and Receiver)
add_library(ltsp_common INTERFACE)
target_sources(ltsp_common INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/common/ltsp_pdu.c
    ${CMAKE_CURRENT_LIST_DIR}/common/ltsp_sequence.c
    ${CMAKE_CURRENT_LIST_DIR}/common/ltsp_min_filter.c
    ${CMAKE_CURRENT_LIST_DIR}/common/ltsp_regression.c
    ${CMAKE_CURRENT_LIST_DIR}/common/ltsp_pio_timestamp.c
)
target_include_directories(ltsp_common INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/common
    ${CMAKE_CURRENT_LIST_DIR}
)

add_subdirectory(gm)
add_subdirectory(receiver)
