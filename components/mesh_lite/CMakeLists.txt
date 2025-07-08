set(srcs "src/esp_mesh_lite_espnow.c")
set(require_components "json" "esp_wifi" "app_update" "wifi_provisioning" "qrcode" "nvs_flash" "protobuf-c" "console")

if (CONFIG_MESH_LITE_ENABLE)
    list(APPEND srcs "src/esp_mesh_lite.c" "src/esp_mesh_lite_port.c" "src/esp_mesh_lite_log.c" "src/mesh_lite.pb-c.c")
endif()

if (CONFIG_MESH_LITE_PROV_ENABLE)
    list(APPEND srcs "src/wifi_prov/wifi_prov_mgr.c" "src/wifi_prov/zero_provisioning.c")
endif()

if (CONFIG_MESH_LITE_WIRELESS_DEBUG)
    list(APPEND srcs "src/esp_mesh_lite_wireless_debug.c")
endif()

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS include
                       REQUIRES ${require_components})


if (NOT CONFIG_MESH_LITE_CORE_ENABLE)
    string(CONCAT LIBS "esp_mesh_lite_" "${IDF_TARGET}")

    add_library(${LIBS} STATIC IMPORTED)

    if (CONFIG_MESH_LITE_PROTO_VERSION_V1X)
        set(REMOVE_FILE esp_mesh_lite_proto_v0_x.c.obj
                        esp_mesh_lite_lan_ota_proto_v0_x.c.obj)
    else()
        set(REMOVE_FILE esp_mesh_lite_proto_v1_x.c.obj
                        esp_mesh_lite_lan_ota_proto_v1_x.c.obj)
    endif()

    execute_process(
        COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/lib/lib${LIBS}.a ${CMAKE_CURRENT_BINARY_DIR}/lib${LIBS}.a
        OUTPUT_VARIABLE AR_OUTPUT
        ERROR_VARIABLE AR_ERROR)

    foreach(file ${REMOVE_FILE})
        execute_process(
            COMMAND ${CMAKE_AR} d "${CMAKE_CURRENT_BINARY_DIR}/lib${LIBS}.a" "${file}"
            OUTPUT_VARIABLE AR_OUTPUT
            ERROR_VARIABLE AR_ERROR)

        execute_process(
            COMMAND ${CMAKE_NM} "${CMAKE_CURRENT_BINARY_DIR}/lib${LIBS}.a"
            OUTPUT_VARIABLE AR_OUTPUT
            ERROR_VARIABLE AR_ERROR)

        string(FIND "${AR_OUTPUT}" "${file}" FOUND_INDEX)
        if(FOUND_INDEX GREATER -1)
            message(FATAL_ERROR
            "- MeshLite protocol version mismatch, terminating build.
            - Please raise an issue at https://github.com/espressif/esp-mesh-lite/issues")
        endif()
    endforeach()

    set_property(TARGET ${LIBS} PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_BINARY_DIR}/lib${LIBS}.a)

    target_link_libraries(${COMPONENT_LIB} INTERFACE ${LIBS})
    set_property(TARGET ${LIBS} APPEND PROPERTY INTERFACE_LINK_LIBRARIES ${COMPONENT_LIB})
endif()

include(package_manager)
if (CONFIG_MESH_LITE_ENABLE)
    cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
endif()
